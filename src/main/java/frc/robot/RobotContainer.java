package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.ScoringLocation.AlgaeHeight;
import frc.lib.util.ScoringLocation.CoralHeight;
import frc.lib.util.ScoringLocation.Height;
import frc.lib.util.ScoringLocation.HeightMode;
import frc.lib.util.viz.FieldViz;
import frc.lib.util.viz.Viz2025;
import frc.robot.Robot.RobotRunType;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberReal;
import frc.robot.subsystems.coral.CoralScoring;
import frc.robot.subsystems.coral.CoralScoringIO;
import frc.robot.subsystems.coral.CoralScoringReal;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorReal;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveIO;
import frc.robot.subsystems.swerve.SwerveReal;
import frc.robot.subsystems.swerve.SwerveSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionReal;
import frc.robot.subsystems.vision.VisionSimPhoton;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    /* Controllers */
    public final CommandXboxController driver = new CommandXboxController(Constants.driverId);
    public final CommandXboxController backUpOperator =
        new CommandXboxController(Constants.operatorId);
    public final CommandXboxController pitController =
        new CommandXboxController(Constants.PIT_CONTROLLER_ID);
    public final CommandXboxController altOperator =
        new CommandXboxController(Constants.ALT_OPERATOR_ID);


    /** Simulation */
    private SwerveDriveSimulation driveSimulation;


    /** Visualization */
    private final FieldViz fieldVis;
    private final Viz2025 vis;
    /** State */
    private final RobotState state;


    /* Subsystems */

    private LEDs leds = new LEDs();
    private Elevator elevator;
    private final Swerve s_Swerve;
    private final Vision s_Vision;
    private CoralScoring coralScoring;
    private Climber climb;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer(RobotRunType runtimeType) {
        fieldVis = new FieldViz();
        vis = new Viz2025(fieldVis, "");
        state = new RobotState(vis);
        switch (runtimeType) {
            case kReal:
                elevator = new Elevator(new ElevatorReal());
                s_Swerve = new Swerve(state, new SwerveReal());
                s_Vision = new Vision(state, VisionReal::new);
                coralScoring = new CoralScoring(new CoralScoringReal());
                climb = new Climber(new ClimberReal());
                break;
            case kSimulation:
                driveSimulation = new SwerveDriveSimulation(Constants.Swerve.getMapleConfig(),
                    new Pose2d(3, 3, Rotation2d.kZero));
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
                s_Swerve = new Swerve(state, new SwerveSim(driveSimulation));
                s_Vision = new Vision(state, VisionSimPhoton.partial(driveSimulation));

                coralScoring = new CoralScoring(new CoralScoringIO() {});
                elevator = new Elevator(new ElevatorIO() {});

                break;
            default:
                elevator = new Elevator(new ElevatorIO() {});
                s_Swerve = new Swerve(state, new SwerveIO.Empty() {});
                s_Vision = new Vision(state, VisionIO::empty);

        }

        /* Default Commands */
        s_Swerve.setDefaultCommand(s_Swerve.teleOpDrive(driver, Constants.Swerve.isFieldRelative,
            Constants.Swerve.isOpenLoop));
        leds.setDefaultCommand(leds.setLEDsBreathe(Color.kRed).ignoringDisable(true));
        /* Button and Trigger Bindings */

        configureButtonBindings(runtimeType);
        configureTriggerBindings();
    }


    /**
     * Use this method to vol your button->command mappings. Buttons can be created by instantiating
     * a {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or
     * {@link XboxController}), and then passing it to a
     * {@link edu1.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings(RobotRunType runtimeType) {
        driver.y().onTrue(new InstantCommand(() -> s_Swerve.resetFieldRelativeOffset()));

        driver.povDown().onTrue(elevator.home());
        driver.povLeft().onTrue(elevator.p0());
        driver.leftTrigger().onTrue(elevator.p2());
        SmartDashboard.putNumber("elevatorVoltage", 1.0);
        driver.povUp().whileTrue(elevator.moveUp());
        driver.povRight().whileTrue(elevator.moveDown());

        // alt operator
        altOperator.povUp().whileTrue(Commands.runOnce(() -> Height.incrementState()));
        altOperator.povDown().whileTrue(Commands.runOnce(() -> Height.decrementState()));
        altOperator.a().whileTrue(elevator.modeSwapper());

        backUpOperator.a().onTrue(new Command() {
            Timer timer = new Timer();

            @Override
            public void initialize() {
                timer.reset();
                timer.start();
            }

            @Override
            public void execute() {
                vis.setElevatorHeight(Inches.of(timer.get() * 30.0));
            }

            @Override
            public boolean isFinished() {
                return timer.hasElapsed(3.0);
            }
        });
        backUpOperator.b().onTrue(new Command() {
            Timer timer = new Timer();

            @Override
            public void initialize() {
                timer.reset();
                timer.start();
            }

            @Override
            public void execute() {
                vis.setElevatorHeight(Inches.of(Math.max(72.0 - timer.get() * 30.0, 0)));
            }

            @Override
            public boolean isFinished() {
                return timer.hasElapsed(3.0);
            }
        });

    }

    /**
     * Triggers
     */

    public void configureTriggerBindings() {
        coralScoring.intakedCoralRight.onTrue(leds.setLEDsSolid(Color.kRed).withTimeout(5));
        coralScoring.intakedCoralRight.onTrue(coralScoring.runPreScoringMotor(2));
        coralScoring.outtakedCoral
            .onTrue(leds.blinkLEDs(LEDPattern.solid(Color.kCyan)).withTimeout(5));
        climb.resetButton.onTrue(climb.restEncoder());
        // driver controls
        driver.x().onTrue(new InstantCommand(() -> {
            s_Swerve.resetOdometry(new Pose2d(7.24, 4.05, Rotation2d.kZero));
        }));
        driver.y().whileTrue(coralScoring.runScoringMotor(2));
        driver.rightBumper().whileTrue(climb.runClimberMotorCommand());

        // alt operator controls
        altOperator.povDown().and(isCoralTrigger)
            .onTrue(Commands.runOnce(() -> CoralHeight.decrementState()));
        altOperator.povUp().and(isCoralTrigger)
            .onTrue(Commands.runOnce(() -> CoralHeight.incrementState()));
        altOperator.povUp().and(isAlgaeTrigger)
            .onTrue(Commands.runOnce(() -> AlgaeHeight.incrementState()));
        altOperator.povDown().and(isAlgaeTrigger)
            .onTrue(Commands.runOnce(() -> AlgaeHeight.decrementState()));
        altOperator.povLeft().onTrue(Commands.runOnce(() -> HeightMode.decrementState()));
        altOperator.povRight().onTrue(Commands.runOnce(() -> HeightMode.incrementState()));
        // altOperator.a().whileTrue(() -> ());

        // pit controller
        pitController.leftBumper().whileTrue(climb.resetClimberCommand());
    }



    // trigger
    public Trigger isCoralTrigger = new Trigger(() -> isCoral());
    public Trigger isAlgaeTrigger = new Trigger(() -> isAlgae());

    public boolean isCoral() {
        return HeightMode.getCurrentHeightMode() == HeightMode.kCoral;
    }

    public boolean isAlgae() {
        return HeightMode.getCurrentHeightMode() == HeightMode.kAlgae;
    }

    /**
     * Gets the user's selected autonomous command.
     *
     * @return Returns autonomous command selected.
     */
    public Command getAutonomousCommand() {
        return null;
    }

    /**
     * Update viz
     */
    public void updateViz() {
        vis.drawImpl();
    }

    /** Start simulation */
    public void startSimulation() {
        if (driveSimulation != null) {
            SimulatedArena.getInstance().resetFieldForAuto();
        }
    }

    /**
     * Update simulation
     */
    public void updateSimulation() {
        if (driveSimulation != null) {
            SimulatedArena.getInstance().simulationPeriodic();
            Logger.recordOutput("FieldSimulation/Algae",
                SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
            Logger.recordOutput("FieldSimulation/Coral",
                SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
            vis.setActualPose(driveSimulation.getSimulatedDriveTrainPose());

        }
    }
}


