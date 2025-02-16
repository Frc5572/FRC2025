package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.ScoringLocation;
import frc.lib.util.viz.FieldViz;
import frc.lib.util.viz.Viz2025;
import frc.robot.Robot.RobotRunType;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberReal;
import frc.robot.subsystems.coral.CoralScoring;
import frc.robot.subsystems.coral.CoralScoringIO;
import frc.robot.subsystems.coral.CoralScoringReal;
import frc.robot.subsystems.coral.CoralScoringSim;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorReal;
import frc.robot.subsystems.elevator.ElevatorSim;
import frc.robot.subsystems.elevator_algae.ElevatorAlgae;
import frc.robot.subsystems.elevator_algae.ElevatorAlgaeIO;
import frc.robot.subsystems.elevator_algae.ElevatorAlgaeReal;
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
    public final CommandXboxController pitController =
        new CommandXboxController(Constants.PIT_CONTROLLER_ID);
    public final CommandXboxController altOperator =
        new CommandXboxController(Constants.ALT_OPERATOR_ID);
    public final CommandXboxController operator = new CommandXboxController(1);


    /** Simulation */
    private SwerveDriveSimulation driveSimulation;
    /** ShuffleBoard */
    public static ShuffleboardTab mainDriverTab = Shuffleboard.getTab("Main Driver");

    public GenericEntry heightState =
        mainDriverTab.add("Elevator Height", ScoringLocation.Height.getCurrentState().displayName)
            .withWidget(BuiltInWidgets.kTextView).withPosition(2, 0).withSize(2, 1).getEntry();
    public GenericEntry heightWidget =
        mainDriverTab.add("Coral Level", ScoringLocation.Height.getCurrentState().ordinal() + 1)
            .withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("min_value ", 1,
                "max_value", 4, "divisions", 4, "Show Text", false, "orientation", "vertical"))
            .withPosition(2, 1).withSize(2, 3).getEntry();

    /** Visualization */
    private final FieldViz fieldVis;
    private final Viz2025 vis;
    /** State */
    private final RobotState state;


    /* Subsystems */
    private ElevatorAlgae algae;
    private LEDs leds = new LEDs();
    private Elevator elevator;
    private final Swerve swerve;
    private final Vision vision;
    private CoralScoring coralScoring;
    private Climber climb;
    private OperatorStates operatorStates = new OperatorStates();

    /* Triggers */
    private Trigger algaeInIntake = new Trigger(() -> algae.hasAlgae());

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer(RobotRunType runtimeType) {
        fieldVis = new FieldViz();
        vis = new Viz2025(fieldVis, "");
        state = new RobotState(vis);
        switch (runtimeType) {
            case kReal:
                elevator = new Elevator(new ElevatorReal(), vis);
                swerve = new Swerve(state, new SwerveReal());
                vision = new Vision(state, VisionReal::new);
                coralScoring = new CoralScoring(new CoralScoringReal(), vis);
                algae = new ElevatorAlgae(new ElevatorAlgaeReal(), vis);
                climb = new Climber(new ClimberReal());
                break;

            case kSimulation:
                driveSimulation = new SwerveDriveSimulation(Constants.Swerve.getMapleConfig(),
                    new Pose2d(3, 3, Rotation2d.kZero));
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
                swerve = new Swerve(state, new SwerveSim(driveSimulation));
                vision = new Vision(state, VisionSimPhoton.partial(driveSimulation));
                elevator = new Elevator(new ElevatorSim(), vis);
                coralScoring = new CoralScoring(new CoralScoringSim(), vis);
                algae = new ElevatorAlgae(new ElevatorAlgaeIO.Empty(), vis);
                climb = new Climber(new ClimberIO.Empty());
                break;
            default:
                elevator = new Elevator(new ElevatorIO.Empty(), vis);
                swerve = new Swerve(state, new SwerveIO.Empty());
                vision = new Vision(state, VisionIO::empty);
                coralScoring = new CoralScoring(new CoralScoringIO.Empty(), vis);
                algae = new ElevatorAlgae(new ElevatorAlgaeIO.Empty(), vis);
                climb = new Climber(new ClimberIO.Empty());
        }

        /* Default Commands */
        leds.setDefaultCommand(leds.setLEDsBreathe(Color.kRed).ignoringDisable(true));
        /* Button and Trigger Bindings */

        configureTriggerBindings();

        if (runtimeType == RobotRunType.kSimulation) {
            maybeController("Driver", driver, this::setupDriver);
        } else {
            setupDriver();
        }
        maybeController("Pit Controller", pitController, this::setupPitController);
        maybeController("Alt Operator", altOperator, this::setupAltOperatorController);
    }

    private List<Runnable> controllerSetups = new ArrayList<>();


    private void maybeController(String name, CommandXboxController xboxController,
        Runnable setupFun) {
        Runnable runner = () -> {
            System.out.println("Setting up buttons for " + name);
            setupFun.run();
        };
        if (xboxController.isConnected()) {
            runner.run();
        } else {
            new Trigger(xboxController::isConnected)
                .onTrue(Commands.runOnce(() -> controllerSetups.add(runner)).ignoringDisable(true));
        }
    }

    /** Setup buttons for newly attached controllers */
    public void queryControllers() {
        for (var setup : controllerSetups) {
            setup.run();
        }
        controllerSetups.clear();
    }

    /**
     * Use this method to vol your button->command mappings. Buttons can be created by instantiating
     * a {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or
     * {@link XboxController}), and then passing it to a
     * {@link edu1.wpi.first.wpilibj2.command.button.JoystickButton}.
     */

    private void setupDriver() {


        swerve.setDefaultCommand(swerve.teleOpDrive(driver, Constants.Swerve.isFieldRelative,
            Constants.Swerve.isOpenLoop));

        driver.y().onTrue(new InstantCommand(() -> swerve.resetFieldRelativeOffset()));

        driver.x().onTrue(new InstantCommand(() -> { // sim only

            swerve.resetOdometry(new Pose2d(7.24, 4.05, Rotation2d.kZero));
        }));
        driver.rightTrigger().and(climb.reachedClimberStart.negate())
            .onTrue(climb
                .runClimberMotorCommand(Constants.Climb.PRE_CLIMB_VOLTAGE,
                    () -> climb.getClimberPosition()
                        .in(Radians) >= Constants.Climb.CLIMBER_OUT_ANGLE.in(Radians))
                .andThen(climb.runClimberMotorCommand(Constants.Climb.RESET_VOLTAGE,
                    () -> climb.getClimberPosition()
                        .in(Radians) <= Constants.Climb.CLIMBER_START_ANGLE.in(Radians))));

        driver.rightTrigger().and(climb.reachedClimberStart)
            .onTrue(climb.runClimberMotorCommand(climb.passedClimbAngle()));
    }

    private void setupAltOperatorController() {
        altOperator.y().onTrue(elevator.home());
        altOperator.x().whileTrue(coralScoring.runScoringMotor(2));
        altOperator.rightTrigger().whileTrue(algae.setMotorVoltageCommand(Constants.Algae.VOLTAGE));
        altOperator.leftTrigger()
            .whileTrue(algae.setMotorVoltageCommand(Constants.Algae.NEGATIVE_VOLTAGE));
        // manual mode
        altOperator.start().onTrue(Commands.runOnce(() -> {
            operatorStates.toggleManualMode();
        }).ignoringDisable(true));
        operatorStates.manualModeCheck.onTrue(elevator.manualMove(altOperator));


        altOperator.a().and(operatorStates.manualModeCheck).whileTrue(elevator.heightSelector());
        altOperator.povUp().and(operatorStates.manualModeCheck)
            .onTrue(Commands.runOnce(() -> frc.lib.util.ScoringLocation.Height.incrementState())
                .ignoringDisable(true));
        altOperator.povDown().and(operatorStates.manualModeCheck)
            .onTrue(Commands.runOnce(() -> frc.lib.util.ScoringLocation.Height.decrementState())
                .ignoringDisable(true));
        altOperator.b().whileTrue(elevator.p0());
    }

    private void setupPitController() {
        pitController.y().whileTrue(climb.resetClimberCommand());
        pitController.leftBumper().whileTrue(climb.resetClimberCommand());
        pitController.x().whileTrue(climb.manualClimb(() -> pitController.getLeftY()));

        // remove later
        SmartDashboard.putNumber("elevatorTargetHeight", 20);
        driver.a().whileTrue(
            elevator.moveTo(() -> Inches.of(SmartDashboard.getNumber("elevatorTargetHeight", 20))));
    }

    private void configureTriggerBindings() {
        coralScoring.coralAtIntake.onTrue(leds.setLEDsSolid(Color.kRed).withTimeout(5));
        // coralScoring.coralAtIntake.onTrue(coralScoring.runPreScoringMotor(2));
        coralScoring.coralOuttaken.onTrue(leds.blinkLEDs(Color.kCyan).withTimeout(5));
        climb.resetButton.onTrue(climb.restEncoder());
        algaeInIntake.onTrue(leds.blinkLEDs(Color.kCyan));
        coralScoring.coralOuttaken.negate().whileTrue(coralScoring.runPreScoringMotor(.1));
        coralScoring.coralOuttaken.onTrue(leds.blinkLEDs(Color.kCyan).withTimeout(5));
        climb.resetButton.and(pitController.y()).onTrue(climb.restEncoder());
        coralScoring.coralOuttaken.onTrue(elevator.p0());
        elevator.elevatorHeight.onTrue(new InstantCommand(() -> swerve.setSpeedMultiplier(0.5)));
        elevator.elevatorHeight.onFalse(new InstantCommand(() -> swerve.setSpeedMultiplier(1)));

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


