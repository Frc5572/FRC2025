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
import frc.lib.util.ScoringLocation.Height;
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
    public final CommandXboxController backUpOperator =
        new CommandXboxController(Constants.operatorId);
    public final CommandXboxController pitController =
        new CommandXboxController(Constants.PIT_CONTROLLER_ID);
    public final CommandXboxController altOperator =
        new CommandXboxController(Constants.ALT_OPERATOR_ID);
    public final CommandXboxController operator = new CommandXboxController(1);


    /** Simulation */
    private SwerveDriveSimulation driveSimulation;
    /** ShuffleBoard */
    public static ShuffleboardTab mainDriverTab = Shuffleboard.getTab("Main Driver");

    public GenericEntry coralState =
        mainDriverTab.add("Coral State", ScoringLocation.CoralHeight.getCurrentState().displayName)
            .withWidget(BuiltInWidgets.kTextView).withPosition(2, 0).withSize(2, 1).getEntry();
    public GenericEntry coralWidget = mainDriverTab
        .add("Coral Level", ScoringLocation.CoralHeight.getCurrentState().ordinal() + 1)
        .withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("min_value ", 1, "max_value",
            4, "divisions", 4, "Show Text", false, "orientation", "vertical"))
        .withPosition(2, 1).withSize(2, 3).getEntry();
    public GenericEntry algaeState = mainDriverTab
        .add("Algae State", ScoringLocation.AlgaeHeight.getCurrentHeightMode().displayName)
        .withWidget(BuiltInWidgets.kTextView).withPosition(4, 0).withSize(2, 1).getEntry();

    public GenericEntry algaeWidget = mainDriverTab
        .add("Algae Level", ScoringLocation.CoralHeight.getCurrentState().ordinal() + 1)
        .withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("min_value ", 1, "max_value",
            2, "divisions", 1, "Show Text", false, "orientation", "vertical"))
        .withPosition(4, 1).withSize(2, 3).getEntry();

    public GenericEntry isCoralMode = RobotContainer.mainDriverTab.add("Elevator Mode", true)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("true_color", 0xffffffff, "false_color", 0xff0af0c3))
        .withPosition(6, 0).withSize(2, 2).getEntry();

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
    // private Trigger coralInIntake = new Trigger(() -> coralScoring.getIntakeBrakeStatus());
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

        new Trigger(() -> elevator.hightNotHome())
            .onTrue(new InstantCommand(() -> swerve.setSpeedMultiplier(0.5)))
            .onFalse(new InstantCommand(() -> swerve.setSpeedMultiplier(1)));

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

        // remove later
        SmartDashboard.putNumber("elevatorTargetHeight", 20);
        driver.a().whileTrue(elevator
            .moveToMagic(() -> Inches.of(SmartDashboard.getNumber("elevatorTargetHeight", 20))));
    }

    private void setupAltOperatorController() {
        // altOperator.povUp().and(HeightMode.coralMode)
        // .onTrue(Commands.runOnce(() -> CoralHeight.incrementState()).ignoringDisable(true));
        // altOperator.povUp().and(HeightMode.algaeMode)
        // .onTrue(Commands.runOnce(() -> AlgaeHeight.incrementState()).ignoringDisable(true));
        // altOperator.povDown().and(HeightMode.algaeMode)
        // .onTrue(Commands.runOnce(() -> AlgaeHeight.decrementState()).ignoringDisable(true));
        // altOperator.povDown().and(HeightMode.coralMode)
        // .onTrue(Commands.runOnce(() -> CoralHeight.decrementState()).ignoringDisable(true));
        // altOperator.povRight()
        // .onTrue(Commands.runOnce(() -> HeightMode.decrementState()).ignoringDisable(true));
        // altOperator.povLeft()
        // .onTrue(Commands.runOnce(() -> HeightMode.incrementState()).ignoringDisable(true));
        altOperator.y().onTrue(elevator.home());
        altOperator.x().whileTrue(coralScoring.runScoringMotor(2));
        altOperator.rightTrigger().whileTrue(algae.setMotorVoltageCommand(Constants.Algae.VOLTAGE));
        altOperator.leftTrigger()
            .whileTrue(algae.setMotorVoltageCommand(Constants.Algae.NEGATIVE_VOLTAGE));

        // altOperator.a().and(HeightMode.algaeMode).and(AlgaeHeight.level1).whileTrue(elevator.p0());
        // altOperator.a().and(HeightMode.algaeMode).and(AlgaeHeight.level2).whileTrue(elevator.p2());
        // altOperator.a().and(HeightMode.coralMode).and(CoralHeight.level1).whileTrue(elevator.p0());
        // altOperator.a().and(HeightMode.coralMode).and(CoralHeight.level2).whileTrue(elevator.p1());
        // altOperator.a().and(HeightMode.coralMode).and(CoralHeight.level3).whileTrue(elevator.p3());
        // altOperator.a().and(HeightMode.coralMode).and(CoralHeight.level4).whileTrue(elevator.p4());

        altOperator.a().whileTrue(elevator.heightSelector());
        altOperator.povUp()
            .onTrue(Commands.runOnce(() -> Height.incrementState()).ignoringDisable(true));
        altOperator.povDown()
            .onTrue(Commands.runOnce(() -> Height.decrementState()).ignoringDisable(true));
        altOperator.b().whileTrue(elevator.p0());
        // altOperator.a().whileTrue(elevator.moveTo(() -> {
        // switch (HeightMode.getCurrentHeightMode()) {
        // case kAlgae:
        // switch (CoralHeight.getCurrentState()) {
        // case Klevel1:
        // return Constants.Elevator.P1;
        // case Klevel2:
        // return Constants.Elevator.P1;

        // case Klevel3:
        // return Constants.Elevator.P1;

        // case Klevel4:
        // return Constants.Elevator.P1;
        // default:
        // return null;
        // }

        // case kCoral:
        // switch (AlgaeHeight.getCurrentHeightMode()) {
        // case Klevel1:
        // return Constants.Elevator.P1;


        // case Klevel2:
        // return Constants.Elevator.P1;
        // default:
        // return null;
        // }
        // default:
        // return null;
        // }
        // }));
    }

    private void setupPitController() {
        pitController.y().whileTrue(climb.resetClimberCommand());
        pitController.leftBumper().whileTrue(climb.resetClimberCommand());
        pitController.x().whileTrue(climb.manualClimb(() -> pitController.getLeftY()));
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
        // coralInIntake.onTrue(elevator.p0());
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


