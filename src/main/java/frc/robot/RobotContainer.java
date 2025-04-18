package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
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
import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.subsystems.algaewrist.AlgaeWrist;
import frc.robot.subsystems.algaewrist.AlgaeWristIO;
import frc.robot.subsystems.algaewrist.AlgaeWristReal;
import frc.robot.subsystems.algaewrist.AlgaeWristSim;
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
import frc.robot.subsystems.swerve.GyroCanandGyro;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroSim;
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
    public static RobotRunType runType = RobotRunType.kReal;

    private final AutoFactory autoFactory;
    private final AutoChooser autoChooser;


    /* Controllers */
    public final CommandXboxController driver = new CommandXboxController(Constants.driverId);
    public final WebController operator = new WebController(5801);
    public final CommandXboxController pitController =
        new CommandXboxController(Constants.PIT_CONTROLLER_ID);
    public final CommandXboxController altOperator =
        new CommandXboxController(Constants.ALT_OPERATOR_ID);
    public final CommandXboxController testController = new CommandXboxController(5);

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
    private LEDs ledsRightSide = new LEDs(buffer, 0, 79);
    private LEDs ledsLeftFrontSide = new LEDs(buffer, 80, 119);
    private LEDs ledsLeftBackSide = new LEDs(buffer, 120, 159);

    private Elevator elevator;
    private final Swerve swerve;
    private final Vision vision;
    private CoralScoring coralScoring;
    private Climber climb;
    private AlgaeWrist wrist;

    Pose2d blueStart = new Pose2d(7.247, 1.126, new Rotation2d(2.276));
    Pose2d redStart = new Pose2d(10.025, 3.476, new Rotation2d(0));

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer(RobotRunType runtimeType) {
        runType = runtimeType;
        fieldVis = new FieldViz();
        vis = new Viz2025(fieldVis, "");
        state = new RobotState(vis);
        leds.setLength(Constants.LEDs.LED_LENGTH);
        leds.start();
        switch (runtimeType) {
            case kReal:
                elevator = new Elevator(new ElevatorReal(), vis);
                swerve = new Swerve(state, new SwerveReal(), new GyroCanandGyro());
                vision = new Vision(state, VisionReal::new);
                coralScoring = new CoralScoring(new CoralScoringReal(), vis);
                algae = new ElevatorAlgae(new ElevatorAlgaeReal(), vis);
                climb = new Climber(new ClimberReal(), vis);
                wrist = new AlgaeWrist(vis, new AlgaeWristReal());
                break;

            case kSimulation:
                driveSimulation =
                    new SwerveDriveSimulation(Constants.Swerve.getMapleConfig(), redStart);
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
                swerve =
                    new Swerve(state, new SwerveSim(driveSimulation), new GyroSim(driveSimulation));
                vision = new Vision(state, VisionSimPhoton.partial(driveSimulation));
                elevator = new Elevator(new ElevatorSim(), vis);
                coralScoring = new CoralScoring(new CoralScoringSim(), vis);
                algae = new ElevatorAlgae(new ElevatorAlgaeIO.Empty(), vis);
                climb = new Climber(new ClimberSim(), vis);
                wrist = new AlgaeWrist(vis, new AlgaeWristSim());
                break;
            default:
                elevator = new Elevator(new ElevatorIO.Empty(), vis);
                swerve = new Swerve(state, new SwerveIO.Empty(), new GyroIO.Empty());
                vision = new Vision(state, VisionIO::empty);
                coralScoring = new CoralScoring(new CoralScoringIO.Empty(), vis);
                algae = new ElevatorAlgae(new ElevatorAlgaeIO.Empty(), vis);
                climb = new Climber(new ClimberIO.Empty(), vis);
                wrist = new AlgaeWrist(vis, new AlgaeWristIO.Empty());
        }
        autoFactory = new AutoFactory(swerve::getPose, swerve::resetOdometry,
            swerve::followTrajectory, true, swerve);

        AutoCommandFactory autos = new AutoCommandFactory(autoFactory, swerve, elevator,
            coralScoring, algae, ledsLeftFrontSide, wrist);
        autoChooser = new AutoChooser();
        autoChooser.addRoutine("Example", autos::example);
        autoChooser.addRoutine("Left Side L4 Coral", autos::l4left);
        autoChooser.addRoutine("Right Side L4 Coral", autos::l4right);
        autoChooser.addRoutine("Middle L4 Coral Right Station", autos::l4middleRightStation);
        autoChooser.addRoutine("Middle L4 Coral Left Station", autos::l4middleLeftStation);
        autoChooser.addRoutine("Front L4 Coral Right Station", autos::l4FrontRightStation);
        autoChooser.addRoutine("Front L4 Coral Left Station", autos::l4FrontLeftStation);
        autoChooser.addRoutine("Barge Right", autos::bargeRight);
        autoChooser.addRoutine("Barge Left", autos::bargeLeft);
        // autoChooser.addRoutine("Barge", autos::barge);
        SmartDashboard.putData(Constants.DashboardValues.autoChooser, autoChooser);

        RobotModeTriggers.autonomous()
            .whileTrue(wrist.homeAngle().andThen(autoChooser.selectedCommandScheduler())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .andThen(Commands.runOnce(() -> swerve.setMotorsZero())));
        RobotModeTriggers.teleop()
            .and(() -> DriverStation.isFMSAttached() || !pitController.isConnected())
            .onTrue(wrist.homeAngle());
        RobotModeTriggers.disabled().onTrue(Commands.runOnce(() -> swerve.setMotorsZero()));


        /* Default Commands */
        ledsRightSide.setDefaultCommand(ledsRightSide.setLEDsBreathe(Color.kRed));
        ledsLeftFrontSide.setDefaultCommand(ledsLeftFrontSide.setLEDsBreathe(Color.kRed));
        ledsLeftBackSide.setDefaultCommand(ledsLeftBackSide.setLEDsBreathe(Color.kRed));

        // algae.setDefaultCommand(algae.algaeHoldCommand().withName("Algae Default Command"));

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

        Command autoScore = CommandFactory
            .autoScore(swerve, elevator, coralScoring, algae, wrist, operator::getDesiredLocation,
                operator::getDesiredHeight, operator::additionalAlgaeHeight, operator::crossOut)
            .andThen(CommandFactory.selectFeeder(swerve, elevator, coralScoring, operator::feeder))
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
        System.out.println("autoscore requires: ");
        for (var req : autoScore.getRequirements()) {
            System.out.println(" - " + req.getName());
        }
        driver.a().and(operator.hasReefLocation()).whileTrue(autoScore)
            .whileTrue(ledsLeftFrontSide.setLEDsSolid(Color.kGreen)
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming))
            .negate().onTrue(coralScoring.runCoralIntake());
        driver.b()
            .whileTrue(CommandFactory.selectFeeder(swerve, elevator, coralScoring, operator::feeder)
                .andThen(swerve.run(() -> {
                })))
            .whileTrue(ledsLeftFrontSide.setLEDsSolid(Color.kGreen)
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        driver.x().onTrue(elevator.home());
        driver.y().onTrue(Commands.runOnce(() -> swerve.resetFieldRelativeOffset()));
        driver.start().and(climb.reachedClimberStart.negate())
            .onTrue(climb
                .runClimberMotorCommand(Constants.Climb.PRE_CLIMB_VOLTAGE,
                    () -> climb.getClimberPosition()
                        .in(Radians) >= Constants.Climb.CLIMBER_OUT_ANGLE.in(Radians))
                .andThen(climb.runClimberMotorCommand(Constants.Climb.RESET_VOLTAGE,
                    () -> climb.getClimberPosition()
                        .in(Radians) <= Constants.Climb.CLIMBER_START_ANGLE.in(Radians))));
        driver.back().whileTrue(climb.runClimberMotorCommand(climb.passedClimbAngle()));
        // driver.leftTrigger()
        // .whileTrue(CommandFactory
        // .doSomethingWithAlgae(swerve, elevator, algae, operator::whatToDoWithAlgae).andThen(
        // CommandFactory.selectFeeder(swerve, elevator, coralScoring, operator::feeder)));
        driver.rightTrigger().whileTrue(elevator.p5().deadlineFor(wrist.bargeAngle()))
            .onFalse(elevator.home().deadlineFor(wrist.homeAngle()));
        driver.back().onTrue(elevator.stop());
        driver.leftTrigger().whileTrue(algae.algaeOuttakeCommand());
        driver.leftBumper().whileTrue(wrist.groundAngle().alongWith(algae.algaeIntakeCommand()))
            .onFalse(wrist.homeAngle().withTimeout(0.5));
        driver.rightBumper().whileTrue(wrist.groundAngle())
            .onFalse(wrist.homeAngle().withTimeout(0.5));
        // driver.leftTrigger().and(() -> operator.whatToDoWithAlgae() == 'd')
        // .whileTrue(algae.algaeOuttakeCommand().withTimeout(1.0));
        // driver.leftTrigger().and(() -> operator.whatToDoWithAlgae() == 'b')
        // .whileTrue(CommandFactory.scoreInBarge(swerve, elevator, algae).andThen(
        // CommandFactory.selectFeeder(swerve, elevator, coralScoring, operator::feeder)));
        // driver.leftTrigger().and(() -> operator.whatToDoWithAlgae() == 'p')
        // .whileTrue(Commands.none());
    }

    private void setupAltOperatorController() {
        altOperator.y().onTrue(elevator.home());
        altOperator.x().and(coralScoring.coralAtOuttake).whileTrue(coralScoring.runCoralOuttake());
        altOperator.rightTrigger().whileTrue(algae.algaeIntakeCommand());
        altOperator.leftTrigger().whileTrue(algae.algaeOuttakeCommand());
        // manual mode

        altOperator.povLeft().onTrue(elevator.moveToFast(() -> Height.KP5.height));

        altOperator.a().whileTrue(elevator.heightSelector());
        altOperator.povUp()
            .onTrue(Commands.runOnce(() -> Height.incrementState()).ignoringDisable(true));
        altOperator.povDown()
            .onTrue(Commands.runOnce(() -> Height.decrementState()).ignoringDisable(true));
        altOperator.b().whileTrue(elevator.p0());
    }

    private void setupPitController() {
        // pitController.a().whileTrue(CommandFactory.scoreInBarge(swerve, elevator, algae, wrist));
        pitController.b().onTrue(elevator.manualMove(altOperator));
        // pitController.leftBumper().whileTrue(climb.resetClimberCommand());
        pitController.x().whileTrue(climb.manualClimb(() -> pitController.getLeftY()));
        pitController.y().onTrue(climb.resetEncoder());
        pitController.start().and(RobotBase::isSimulation).onTrue(
            Commands.runOnce(() -> swerve.resetOdometry(new Pose2d(7.24, 4.05, Rotation2d.kZero))));
        // remove later
        SmartDashboard.putNumber("elevatorTargetHeight", 20);

        SmartDashboard.putNumber("wristVoltage", 0.5);

        pitController.leftBumper()
            .whileTrue(wrist.runVolts(() -> SmartDashboard.getNumber("wristVoltage", 0)))
            .onFalse(wrist.runVolts(() -> 0.0).withTimeout(0.5));
        pitController.rightBumper().whileTrue(wrist.coast());
        pitController.a().whileTrue(wrist.goToAngle(() -> Degrees.of(-22)))
            .onFalse(wrist.runVolts(() -> 0.0));
        // driver.a().whileTrue(
        // elevator.moveTo(() -> Inches.of(SmartDashboard.getNumber("elevatorTargetHeight", 20))));
    }

    private void configureTriggerBindings() {
        // Coral
        coralScoring.coralAtIntake.whileTrue(ledsLeftBackSide.setLEDsSolid(Color.kOrange))
            .whileTrue(ledsLeftFrontSide.setLEDsSolid(Color.kOrange));
        coralScoring.coralAtOuttake.whileTrue(ledsLeftBackSide.setLEDsSolid(Color.kCyan))
            .whileTrue(ledsLeftFrontSide.setLEDsSolid(Color.kCyan));
        vision.seesTwoAprilTags.whileTrue(ledsRightSide.setRainbow());

        coralScoring.coralAtOuttake.negate().debounce(1.0).whileTrue(coralScoring.runCoralIntake());
        RobotModeTriggers.disabled().whileFalse(coralScoring.runCoralIntake());
        // Algae
        // algae.hasAlgae.and(coralScoring.coralAtOuttake.negate())
        // .onTrue(ledsrightbackside.blinkLEDs(Color.kGreen, 2));
        // Climb
        elevator.hightAboveP0.or(climb.reachedClimberStart)
            .onTrue(Commands.runOnce(() -> swerve.setSpeedMultiplier(0.15)).ignoringDisable(true))
            .onFalse(Commands.runOnce(() -> swerve.setSpeedMultiplier(1.0)).ignoringDisable(true));
        RobotModeTriggers.disabled().and(vision.seesTwoAprilTags).whileTrue(
            Commands.run(() -> swerve.resetFieldRelativeOffsetBasedOnPose()).ignoringDisable(true));
        elevator.heightAboveHome.onFalse(algae.setSpeedMultiplier(.25))
            .onTrue(algae.setSpeedMultiplier(1.0));
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

    public void periodic() {
        leds.setData(buffer);
    }
}
