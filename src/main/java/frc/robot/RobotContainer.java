package frc.robot;

import static edu.wpi.first.units.Units.Radians;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.ScoringLocation.Height;
import frc.lib.util.WebController;
import frc.lib.util.viz.FieldViz;
import frc.lib.util.viz.Viz2025;
import frc.robot.Robot.RobotRunType;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberReal;
import frc.robot.subsystems.climber.ClimberSim;
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
    private final AutoFactory autoFactory;
    private final AutoChooser autoChooser;


    /* Controllers */
    public final CommandXboxController driver = new CommandXboxController(Constants.driverId);
    public final WebController operator = new WebController(5801);
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
    private ElevatorAlgae algae;
    private final AddressableLED leds = new AddressableLED(Constants.LEDs.LED_PORT);
    private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(Constants.LEDs.LED_LENGTH);
    private LEDs ledsright = new LEDs(buffer, leds, 0, 59);
    private LEDs ledsleft = new LEDs(buffer, leds, 60, 119);
    private Elevator elevator;
    private final Swerve swerve;
    private final Vision vision;
    private CoralScoring coralScoring;
    private Climber climb;
    private OperatorStates operatorStates = new OperatorStates();

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
                climb = new Climber(new ClimberReal(), vis);
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
                climb = new Climber(new ClimberSim(), vis);
                break;
            default:
                elevator = new Elevator(new ElevatorIO.Empty(), vis);
                swerve = new Swerve(state, new SwerveIO.Empty());
                vision = new Vision(state, VisionIO::empty);
                coralScoring = new CoralScoring(new CoralScoringIO.Empty(), vis);
                algae = new ElevatorAlgae(new ElevatorAlgaeIO.Empty(), vis);
                climb = new Climber(new ClimberIO.Empty(), vis);
        }
        autoFactory = new AutoFactory(swerve::getPose, swerve::resetOdometry,
            swerve::followTrajectory, true, swerve);

        AutoCommandFactory autos =
            new AutoCommandFactory(autoFactory, swerve, elevator, coralScoring, algae, ledsleft);
        autoChooser = new AutoChooser();
        autoChooser.addRoutine("Example", autos::example);
        autoChooser.addRoutine("TestAuto", autos::test);
        autoChooser.addRoutine("TestAutoRight", autos::test2);
        autoChooser.addRoutine("Barge", autos::barge);


        SmartDashboard.putData("Dashboard/Auto/Auto Chooser", autoChooser);
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler()
            .andThen(Commands.runOnce(() -> swerve.setMotorsZero())));
        RobotModeTriggers.disabled().onTrue(Commands.runOnce(() -> swerve.setMotorsZero()));


        /* Default Commands */
        ledsright.setDefaultCommand(ledsright.setLEDsBreathe(Color.kRed));
        ledsleft.setDefaultCommand(ledsleft.setLEDsBreathe(Color.kRed));

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
    private final Set<String> seenController = new HashSet<>();

    private void maybeController(String name, CommandXboxController xboxController,
        Runnable setupFun) {
        Runnable runner = () -> {
            if (seenController.add(name)) {
                System.out.println("Setting up buttons for " + name);
                setupFun.run();
            }
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
        operator.periodic();
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

        driver.y().onTrue(Commands.runOnce(() -> swerve.resetFieldRelativeOffset()));
        driver.rightTrigger().and(climb.reachedClimberStart.negate())
            .onTrue(climb
                .runClimberMotorCommand(Constants.Climb.PRE_CLIMB_VOLTAGE,
                    () -> climb.getClimberPosition()
                        .in(Radians) >= Constants.Climb.CLIMBER_OUT_ANGLE.in(Radians))
                .andThen(climb.runClimberMotorCommand(Constants.Climb.RESET_VOLTAGE,
                    () -> climb.getClimberPosition()
                        .in(Radians) <= Constants.Climb.CLIMBER_START_ANGLE.in(Radians))));

        driver.rightTrigger().and(climb.reachedClimberStart)
            .whileTrue(climb.runClimberMotorCommand(climb.passedClimbAngle()));
        driver.a().and(operator.hasReefLocation())
            .whileTrue(CommandFactory
                .autoScore(swerve, elevator, coralScoring, algae, operator::getDesiredLocation,
                    operator::getDesiredHeight)
                .andThen(CommandFactory.selectFeeder(swerve, elevator, operator::feeder))
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming))
            .negate().onTrue(coralScoring.runCoralIntake());
        driver.b().whileTrue(CommandFactory.selectFeeder(swerve, elevator, operator::feeder)
            .andThen(swerve.run(() -> {
            })));
    }

    private void setupAltOperatorController() {
        altOperator.y().onTrue(elevator.home());
        altOperator.x().and(coralScoring.coralAtOuttake).whileTrue(coralScoring.runCoralOuttake());
        altOperator.rightTrigger().whileTrue(algae.algaeIntakeCommand());
        altOperator.leftTrigger().whileTrue(algae.runAlgaeMotor(Constants.Algae.NEGATIVE_VOLTAGE));
        // manual mode

        altOperator.povLeft().onTrue(elevator.moveTo(() -> Height.KP5.height));

        altOperator.a().whileTrue(elevator.heightSelector());
        altOperator.povUp()
            .onTrue(Commands.runOnce(() -> Height.incrementState()).ignoringDisable(true));
        altOperator.povDown()
            .onTrue(Commands.runOnce(() -> Height.decrementState()).ignoringDisable(true));
        altOperator.b().whileTrue(elevator.p0());
    }

    private void setupPitController() {
        pitController.b().onTrue(elevator.manualMove(altOperator));
        pitController.y().whileTrue(climb.resetClimberCommand());
        pitController.leftBumper().whileTrue(climb.resetClimberCommand());
        pitController.x().whileTrue(climb.manualClimb(() -> pitController.getLeftY()));
        pitController.start().and(RobotBase::isSimulation).onTrue(
            Commands.runOnce(() -> swerve.resetOdometry(new Pose2d(7.24, 4.05, Rotation2d.kZero))));
        // remove later
        SmartDashboard.putNumber("elevatorTargetHeight", 20);
        // driver.a().whileTrue(
        // elevator.moveTo(() -> Inches.of(SmartDashboard.getNumber("elevatorTargetHeight", 20))));
        climb.resetButton.and(pitController.y()).onTrue(climb.resetEncoder());
    }

    private void configureTriggerBindings() {
        // Coral
        coralScoring.coralAtIntake.whileTrue(
            ledsright.setLEDsSolid(Color.kBlue).alongWith(ledsleft.setLEDsSolid(Color.kBlue)));
        coralScoring.coralAtOuttake.whileTrue(
            ledsright.setLEDsSolid(Color.kCyan).alongWith(ledsleft.setLEDsSolid(Color.kCyan)));
        vision.seesTwoAprilTags.whileTrue(ledsright.setRainbow().alongWith(ledsleft.setRainbow()));


        coralScoring.coralAtOuttake.negate().debounce(1.0).whileTrue(coralScoring.runCoralIntake());
        RobotModeTriggers.disabled().whileFalse(coralScoring.runCoralIntake());
        // Algae
        algae.hasAlgae.and(coralScoring.coralAtOuttake.negate()).onTrue(
            ledsright.blinkLEDs(Color.kCyan, 2).alongWith(ledsleft.blinkLEDs(Color.kCyan, 2)));
        // Climb
        climb.resetButton.onTrue(climb.resetEncoder());
        // coralScoring.coralAtOuttake.and(RobotModeTriggers.teleop()).onTrue(elevator.p0());
        elevator.hightAboveP0.or(climb.reachedClimberStart)
            .onTrue(Commands.runOnce(() -> swerve.setSpeedMultiplier(0.15)).ignoringDisable(true))
            .onFalse(Commands.runOnce(() -> swerve.setSpeedMultiplier(1.0)).ignoringDisable(true));

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
