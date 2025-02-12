package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import java.util.ArrayList;
import java.util.List;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.ScoringLocation.HeightMode;
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
    public final CommandXboxController backUpOperator =
        new CommandXboxController(Constants.operatorId);
    public final CommandXboxController pitController =
        new CommandXboxController(Constants.PIT_CONTROLLER_ID);
    public final CommandXboxController altOperator =
        new CommandXboxController(Constants.ALT_OPERATOR_ID);
    public final CommandXboxController operator = new CommandXboxController(1);


    /** Simulation */
    private SwerveDriveSimulation driveSimulation;


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

    /* Triggers */
    private Trigger algaeInIntake = new Trigger(() -> algae.hasAlgae());

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
                elevator = new Elevator(new ElevatorReal(), vis);
                swerve = new Swerve(state, new SwerveReal());
                vision = new Vision(state, VisionReal::new);
                coralScoring = new CoralScoring(new CoralScoringReal(), vis);
                algae = new ElevatorAlgae(new ElevatorAlgaeReal(), vis);
                climb = new Climber(new ClimberReal());
                break;
            case kSimulation:
                driveSimulation = new SwerveDriveSimulation(Constants.Swerve.getMapleConfig(),
                    new Pose2d(1, 3, Rotation2d.kZero));
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
                swerve = new Swerve(state, new SwerveSim(driveSimulation));
                vision = new Vision(state, VisionSimPhoton.partial(driveSimulation));
                elevator = new Elevator(new ElevatorSim(), vis);
                coralScoring = new CoralScoring(new CoralScoringIO.Empty(), vis);
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

    private void setupDriver() {
        swerve.setDefaultCommand(swerve.teleOpDrive(driver, Constants.Swerve.isFieldRelative,
            Constants.Swerve.isOpenLoop));
    }

    private void setupAltOperatorController() {
        altOperator.a().onTrue(swerve.moveAndAvoidReef(() -> {
            return new Pose2d(1.06, 1.01, Rotation2d.fromDegrees(52.67));
        }, false, 0.1, 2).andThen(Commands.runOnce(() -> {
            System.out.println("done");
        })).andThen(swerve.stop()));
        altOperator.b().onTrue(swerve.moveAndAvoidReef(() -> {
            return new Pose2d(5.04, 5.31, Rotation2d.fromDegrees(-120.02));
        }, false, 0.1, 2).andThen(Commands.runOnce(() -> {
            System.out.println("done");
        })).andThen(swerve.stop()));
        altOperator.a().negate().and(altOperator.b().negate()).whileTrue(swerve.stop());
    }

    private void setupPitController() {

    }

    private void configureTriggerBindings() {
        coralScoring.intakedCoralRight.onTrue(leds.setLEDsSolid(Color.kRed).withTimeout(5));
        coralScoring.intakedCoralRight.onTrue(coralScoring.runPreScoringMotor(2));
        coralScoring.outtakedCoral.onTrue(leds.blinkLEDs(Color.kCyan).withTimeout(5));
        climb.resetButton.onTrue(climb.restEncoder());
        algaeInIntake.onTrue(leds.blinkLEDs(Color.kCyan));
        coralScoring.outtakedCoral.negate().whileTrue(coralScoring.runPreScoringMotor(.1));
        coralScoring.outtakedCoral.onTrue(leds.blinkLEDs(Color.kCyan).withTimeout(5));
        climb.resetButton.and(pitController.y()).onTrue(climb.restEncoder());
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
        return swerve.moveAndAvoidReef(
            () -> new Pose2d(FieldConstants.fieldLength.in(Meters) - 1, 3, Rotation2d.kZero), false,
            0.1, 1);
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
            // SimulatedArena.getInstance().resetFieldForAuto();
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


