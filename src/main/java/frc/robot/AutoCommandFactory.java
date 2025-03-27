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
import frc.lib.util.ScoringLocation;
import frc.lib.util.ScoringLocation.CoralLocation;
import frc.lib.util.ScoringLocation.Height;
import frc.robot.commands.MoveAndAvoidReef;
import frc.robot.commands.MoveToPose;
import frc.robot.subsystems.LEDs;
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
        CoralScoring coral, ElevatorAlgae algae, LEDs leds) {
        this.autoFactory = autoFactory;
        this.swerve = swerve;
        this.elevator = elevator;
        this.coral = coral;
        this.algae = algae;
        this.leds = leds;

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

    private static final Pose2d middleStart =
        new Pose2d(7.578684329986572, 3.9847824573516846, Rotation2d.k180deg);

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
        Command ret = CommandFactory.dropAlgaeIntake(swerve);
        for (var loc : locations) {
            ret = ret.andThen(CommandFactory.autoScore(swerve, elevator, coral, algae, () -> loc,
                () -> Height.KP4, () -> Optional.empty(), (x) -> {
                }));
            if (isLeft) {
                ret = ret.andThen(CommandFactory.leftFeeder(swerve, elevator, coral));
            } else {
                ret = ret.andThen(CommandFactory.rightFeeder(swerve, elevator, coral));
            }
            ret = ret.andThen(coral.runCoralIntake().until(coral.coralAtIntake));
        }
        ret = ret.andThen(swerve.stop());
        routine.active().onTrue(ret.alongWith(algae.algaeOuttakeCommand().withTimeout(5.0))
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
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
            ret = ret.andThen(CommandFactory.autoScore(swerve, elevator, coral, algae, () -> loc,
                () -> Height.KP4, () -> Optional.empty(), (x) -> {
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
