package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import java.util.Optional;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.AllianceFlipUtil;
import frc.lib.util.ScoringLocation;
import frc.lib.util.ScoringLocation.CoralLocation;
import frc.lib.util.ScoringLocation.Height;
import frc.robot.commands.MoveAndAvoidReef;
import frc.robot.commands.MoveToPose;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.algaewrist.AlgaeWrist;
import frc.robot.subsystems.coral.CoralScoring;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator_algae.ElevatorAlgae;
import frc.robot.subsystems.swerve.Swerve;

/**
 * Command Factory for Autos
 */
public class AutoCommandFactory {

    AutoFactory autoFactory;
    Swerve swerve;
    Elevator elevator;
    CoralScoring coral;
    LEDs leds;
    ElevatorAlgae algae;
    AlgaeWrist wrist;

    private SendableChooser<ScoringLocation.Height> algaeHeight =
        new SendableChooser<ScoringLocation.Height>();

    /**
     * Command Factory for Autos
     *
     * @param autoFactory Choreo Auto Factory
     * @param swerve Swerve Subsystem
     * @param elevator Elevator Subsystem
     * @param coral Coral Subsystem
     * @param leds LED Subsystem
     */
    public AutoCommandFactory(AutoFactory autoFactory, Swerve swerve, Elevator elevator,
        CoralScoring coral, ElevatorAlgae algae, LEDs leds, AlgaeWrist wrist) {
        this.autoFactory = autoFactory;
        this.swerve = swerve;
        this.elevator = elevator;
        this.coral = coral;
        this.algae = algae;
        this.leds = leds;
        this.wrist = wrist;

        algaeHeight.setDefaultOption("High", ScoringLocation.Height.KP2);
        algaeHeight.addOption("Low", ScoringLocation.Height.KP0);
        SmartDashboard.putData(Constants.DashboardValues.algaeHeight, algaeHeight);
    }

    /**
     * Example Auto Routine
     *
     * @return Auto Routine
     */
    public AutoRoutine example() {
        AutoRoutine routine = autoFactory.newRoutine("Example");
        MoveToPose testMTP =
            new MoveToPose(swerve,
                () -> new Pose2d(FieldConstants.Reef.center.getX(),
                    FieldConstants.Barge.middleCage.getY(), new Rotation2d()),
                true, .25, 10, routine);

        routine.active()
            .onTrue(Commands.sequence(
                Commands.runOnce(() -> swerve
                    .resetOdometry(new Pose2d(FieldConstants.Barge.middleCage, new Rotation2d()))),
                leds.blinkLEDs(Color.kGreen, 5), testMTP));
        testMTP.done().onTrue(leds.blinkLEDs(Color.kPurple, 5));
        return routine;
    }

    /**
     * Left L4 Auto
     *
     * @return Auto Routine
     */
    public AutoRoutine l4left() {
        return scoreCoral("l4left", true, CoralLocation.I, CoralLocation.K, CoralLocation.A,
            CoralLocation.L);
    }

    /**
     * Right L4 Auto
     *
     * @return Auto Routine
     */
    public AutoRoutine l4right() {
        return scoreCoral("l4right", false, CoralLocation.E, CoralLocation.C, CoralLocation.D,
            CoralLocation.B);
    }

    /**
     * Middle L4 Auto using Right Station
     */
    public AutoRoutine l4middleRightStation() {
        return scoreWideBerthCoral("l4middleRightStation", false, false, CoralLocation.G,
            CoralLocation.H);
    }

    /**
     * Middle L4 Auto using Left Station
     */
    public AutoRoutine l4middleLeftStation() {
        return scoreWideBerthCoral("l4middleLeftStation", false, true, CoralLocation.G,
            CoralLocation.H);
    }

    /**
     * Go around and score the front using Right Station
     */
    public AutoRoutine l4FrontRightStation() {
        return scoreWideBerthCoral("l4frontRightStation", true, false, CoralLocation.A,
            CoralLocation.B);
    }

    /**
     * Middle L4 Auto using Left Station
     */
    public AutoRoutine l4FrontLeftStation() {
        return scoreWideBerthCoral("l4frontLeftStation", true, true, CoralLocation.A,
            CoralLocation.B);
    }

    private AutoRoutine scoreCoral(String name, boolean isLeft, CoralLocation... locations) {
        AutoRoutine routine = autoFactory.newRoutine(name);
        Command ret = CommandFactory.dropAlgaeIntake(swerve).deadlineFor(coral.runCoralIntake());
        for (var loc : locations) {
            ret = ret.andThen(CommandFactory.autoScore(swerve, elevator, coral, algae, wrist,
                () -> loc, () -> Height.KP4, () -> Optional.empty(), (x) -> {
                }));
            if (isLeft) {
                ret = ret.andThen(
                    CommandFactory.leftFeeder(swerve, elevator, coral).until(coral.coralAtIntake));
            } else {
                ret = ret.andThen(
                    CommandFactory.rightFeeder(swerve, elevator, coral).until(coral.coralAtIntake));
            }
            ret = ret.andThen(coral.runCoralIntake().until(coral.coralAtIntake));
        }
        ret = ret.andThen(swerve.stop());
        routine.active().onTrue(ret.withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        return routine;
    }

    private static final Pose2d wideBerthFar = new Pose2d(6.259715557098389, 6.828808784484863,
        Rotation2d.fromRadians(-2.219791763555776));

    private static final Pose2d wideBerthClose =
        new Pose2d(0.5716622471809387, 5.0152268409729, Rotation2d.kZero);

    private AutoRoutine scoreWideBerthCoral(String name, boolean wideBerthFirst, boolean isLeft,
        CoralLocation... locations) {
        AutoRoutine routine = autoFactory.newRoutine(name);
        Command ret = CommandFactory.dropAlgaeIntake(swerve);
        boolean isFirst = true;
        for (var loc : locations) {
            if (!isFirst || wideBerthFirst) {
                Pose2d berthPose =
                    (isFirst || loc.pose.getX() > FieldConstants.Reef.center.getX()) ? wideBerthFar
                        : wideBerthClose;
                if (isLeft) {
                    ret = ret.andThen(new MoveAndAvoidReef(swerve, () -> berthPose,
                        () -> Constants.SwerveTransformPID.MAX_VELOCITY, true,
                        Units.inchesToMeters(24), 180));
                } else {
                    ret = ret.andThen(new MoveAndAvoidReef(swerve,
                        () -> new Pose2d(berthPose.getX(),
                            FieldConstants.fieldWidth.in(Meters) - berthPose.getY(),
                            berthPose.getRotation().unaryMinus()),
                        () -> Constants.SwerveTransformPID.MAX_VELOCITY, true,
                        Units.inchesToMeters(24), 180));
                }
            }
            isFirst = false;
            ret = ret.andThen(CommandFactory.autoScore(swerve, elevator, coral, algae, wrist,
                () -> loc, () -> Height.KP4, () -> Optional.empty(), (x) -> {
                }));
            boolean isFar = loc.pose.getX() > FieldConstants.Reef.center.getX();
            Command feederCommand;
            Command wideBerthCommand;
            Pose2d berthPose = isFar ? wideBerthFar : wideBerthClose;
            if (isLeft) {
                if (isFar) {
                    feederCommand = CommandFactory.leftFeeder(swerve, elevator, coral);
                } else {
                    feederCommand = CommandFactory.leftFeederClose(swerve, elevator, coral);
                }
                wideBerthCommand = new MoveAndAvoidReef(swerve, () -> berthPose,
                    () -> Constants.SwerveTransformPID.MAX_VELOCITY, true, Units.inchesToMeters(24),
                    180, routine);
            } else {
                if (isFar) {
                    feederCommand = CommandFactory.rightFeeder(swerve, elevator, coral);
                } else {
                    feederCommand = CommandFactory.rightFeederClose(swerve, elevator, coral);
                }
                wideBerthCommand = new MoveAndAvoidReef(swerve,
                    () -> new Pose2d(berthPose.getX(),
                        FieldConstants.fieldWidth.in(Meters) - berthPose.getY(),
                        berthPose.getRotation().unaryMinus()),
                    () -> Constants.SwerveTransformPID.MAX_VELOCITY, true, Units.inchesToMeters(24),
                    180, routine);
            }
            ret = ret.andThen(
                wideBerthCommand.alongWith(Commands.waitSeconds(0.2)
                    .andThen(CommandFactory.ensureHome(elevator).withTimeout(1.0))),
                feederCommand, coral.runCoralIntake().until(coral.coralAtIntake));
        }
        ret = ret.andThen(swerve.stop());
        routine.active().onTrue(ret.alongWith(algae.algaeOuttakeCommand().withTimeout(5.0))
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        return routine;
    }

    /** Score one coral, 2 barge */
    public AutoRoutine bargeRight() {
        return coralThenBarge("bargeRight", ScoringLocation.CoralLocation.H, AlgaeLocation.D,
            AlgaeLocation.C);
    }

    /** Score one coral, 2 barge */
    public AutoRoutine bargeLeft() {
        return coralThenBarge("bargeLeft", ScoringLocation.CoralLocation.H, AlgaeLocation.D,
            AlgaeLocation.E);
    }

    private static enum AlgaeLocation {
        // @formatter:off
        A(ScoringLocation.CoralLocation.B, ScoringLocation.Height.KP2),
        B(ScoringLocation.CoralLocation.D, ScoringLocation.Height.KP0),
        C(ScoringLocation.CoralLocation.F, ScoringLocation.Height.KP2),
        D(ScoringLocation.CoralLocation.H, ScoringLocation.Height.KP0),
        E(ScoringLocation.CoralLocation.J, ScoringLocation.Height.KP2),
        F(ScoringLocation.CoralLocation.L, ScoringLocation.Height.KP0);
        // @formatter:on

        public final ScoringLocation.CoralLocation location;
        public final ScoringLocation.Height height;

        AlgaeLocation(ScoringLocation.CoralLocation location, ScoringLocation.Height height) {
            this.location = location;
            this.height = height;
        }
    }

    private static final Pose2d bargePose = new Pose2d(7.558475971221924 + Units.inchesToMeters(0),
        6.258963108062744, Rotation2d.kZero);

    private static final Pose2d middleStartLocation = new Pose2d(
        FieldConstants.startingLineX.in(Meters) - Constants.Swerve.bumperFront.in(Meters),
        FieldConstants.fieldWidth.in(Meters) / 2, Rotation2d.k180deg);

    private AutoRoutine coralThenBarge(String name,
        ScoringLocation.CoralLocation coralScoreLocation, AlgaeLocation... algaeScoreLocations) {
        AutoRoutine routine = autoFactory.newRoutine(name);

        Command run =
            swerve.runOnce(() -> swerve.resetOdometry(AllianceFlipUtil.apply(middleStartLocation)))
                .andThen(CommandFactory.scoreCoralAutoStart(swerve, elevator, coral, algae, wrist,
                    () -> coralScoreLocation, () -> ScoringLocation.Height.KP4));

        for (var algaeLoc : algaeScoreLocations) {
            run = run.andThen(
                CommandFactory.reefPreAlign(swerve, () -> algaeLoc.location)
                    .deadlineFor(elevator.home(), wrist.homeAngle()),
                CommandFactory.maybePickupAlgae(swerve, elevator, algae, wrist,
                    () -> algaeLoc.location, () -> algaeLoc.height, (x) -> {
                    }),
                new MoveAndAvoidReef(swerve,
                    () -> new Pose2d(bargePose.getTranslation(),
                        FieldConstants.Reef.center.minus(bargePose.getTranslation()).getAngle()),
                    () -> Constants.SwerveTransformPID.MAX_VELOCITY, true, Units.inchesToMeters(36),
                    180, routine).deadlineFor(CommandFactory.ensureHome(elevator)),
                new MoveToPose(swerve, () -> bargePose,
                    () -> Constants.SwerveTransformPID.MAX_ELEVATOR_UP_VELOCITY, true,
                    Units.inchesToMeters(6), 15, routine),
                elevator.p5().andThen(algae.algaeOuttakeCommand().withTimeout(0.7))
                    .deadlineFor(wrist.bargeAngle()),
                new MoveToPose(swerve,
                    () -> new Pose2d(bargePose.getTranslation(),
                        FieldConstants.Reef.center.minus(bargePose.getTranslation()).getAngle()),
                    () -> Constants.SwerveTransformPID.MAX_ELEVATOR_UP_VELOCITY, true,
                    Units.inchesToMeters(180), 15, routine)
                        .deadlineFor(CommandFactory.ensureHome(elevator)));
        }

        run = run.andThen(swerve.stop(), elevator.home());
        routine.active().onTrue(run.withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

        return routine;
    }

    // /** Barge auto */
    // public AutoRoutine barge() {

    // AutoRoutine routine = autoFactory.newRoutine("Barge");

    // routine.active().onTrue(Commands.sequence(swerve.runOnce(() -> {
    // swerve.resetOdometry(AllianceFlipUtil.apply(middleStart));
    // }), CommandFactory.autoScore(swerve, elevator, coral, algae, () -> CoralLocation.H,
    // () -> algaeHeight.getSelected(), () -> Optional.empty(), intakingAlgae, (x) -> {
    // }), CommandFactory.autoScore(swerve, elevator, coral, algae, () -> CoralLocation.H,
    // () -> Height.KP4, () -> Optional.empty(), intakingAlgae, (x) -> {
    // }),
    // CommandFactory.barge(swerve, elevator),
    // elevator.moveTo(() -> ScoringLocation.Height.KP5.height),
    // algae.runAlgaeMotor(Constants.Algae.VOLTAGE).withDeadline(Commands.waitSeconds(.25)),
    // CommandFactory.ensureHome(elevator)
    // .andThen(CommandFactory.autoScore(swerve, elevator, coral, algae,
    // () -> CoralLocation.J, () -> algaeHeight.getSelected(), () -> Optional.empty(),
    // intakingAlgae, (x) -> {
    // })),
    // CommandFactory.ensureHome(elevator).andThen(CommandFactory.barge(swerve, elevator)),
    // elevator.moveTo(() -> ScoringLocation.Height.KP5.height),
    // algae.runAlgaeMotor(Constants.Algae.VOLTAGE).withDeadline(Commands.waitSeconds(.5)),
    // CommandFactory.ensureHome(elevator), swerve.stop()));
    // return routine;
    // }

}
