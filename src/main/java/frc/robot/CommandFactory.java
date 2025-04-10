package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.lib.util.AllianceFlipUtil;
import frc.lib.util.ScoringLocation;
import frc.robot.commands.MoveAndAvoidReef;
import frc.robot.commands.MoveToPose;
import frc.robot.subsystems.coral.CoralScoring;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator_algae.ElevatorAlgae;
import frc.robot.subsystems.swerve.Swerve;

/**
 * Factory for Composed Commands
 */
public class CommandFactory {

    /** Move slightly to ensure algae intake is dropped. */
    public static Command dropAlgaeIntake(Swerve swerve) {
        return swerve.run(() -> {
            swerve.setModuleStates(new ChassisSpeeds(4.0, 0.0, 0.0));
        }).withTimeout(0.2).andThen(swerve.stop()).andThen(Commands.waitSeconds(0.25));
    }

    /** Approach reef scoring location. Elevator can be home during this. */
    public static Command reefPreAlign(Swerve swerve,
        Supplier<ScoringLocation.CoralLocation> location) {
        return new MoveAndAvoidReef(swerve, () -> {
            Pose2d finalLoc = location.get().pose;
            return new Pose2d(
                finalLoc.getTranslation()
                    .minus(new Translation2d(Units.inchesToMeters(12), finalLoc.getRotation())),
                finalLoc.getRotation().plus(Rotation2d.fromDegrees(10)));
        }, () -> Constants.SwerveTransformPID.MAX_VELOCITY, true, Units.inchesToMeters(24), 15);
    }

    /** Align to a given scoring location, assuming elevator is at the right height. */
    public static Command reefAlign(Swerve swerve, Supplier<ScoringLocation.CoralLocation> location,
        double distAway) {
        return new MoveToPose(swerve, () -> {
            Pose2d finalLoc = location.get().pose;

            return new Pose2d(
                finalLoc.getTranslation().minus(
                    new Translation2d(Units.inchesToMeters(distAway), finalLoc.getRotation())),
                finalLoc.getRotation());
        }, () -> Constants.SwerveTransformPID.MAX_ELEVATOR_UP_VELOCITY, true,
            Units.inchesToMeters(0.25), 0.2);
    }

    /** Safely move far away enough to go home. */
    public static Command backAwayReef(Swerve swerve,
        Supplier<ScoringLocation.CoralLocation> location) {
        return new MoveToPose(swerve, () -> {
            Pose2d finalLoc = location.get().pose;
            return new Pose2d(
                finalLoc.getTranslation()
                    .minus(new Translation2d(Units.inchesToMeters(16), finalLoc.getRotation())),
                finalLoc.getRotation());
        }, () -> Constants.SwerveTransformPID.MAX_ELEVATOR_UP_VELOCITY, true,
            Units.inchesToMeters(4), 5).withTimeout(1.5);
    }

    public static Command scoreWithElevator(Swerve swerve, Elevator elevator,
        Supplier<ScoringLocation.CoralLocation> location, Supplier<ScoringLocation.Height> height) {
        return reefAlign(swerve, location, 1).alongWith(elevator.follow(() -> {
            Pose2d pose = location.get().pose;
            Translation2d q = pose.getTranslation().minus(swerve.getPose().getTranslation());
            Distance proj = Meters.of(
                q.getX() * pose.getRotation().getCos() + q.getY() * pose.getRotation().getSin());
            Distance adjHeight = Inches.of(Constants.Elevator.HEIGHT_PER_METER_AWAY.in(Inches)
                * MathUtil.clamp(proj.in(Inches), 0, 3) + height.get().height.in(Inches));
            return adjHeight;
        }));
    }

    /** Go home, no exception */
    public static Command ensureHome(Elevator elevator) {
        return elevator.home().repeatedly().until(() -> elevator.getHeight().in(Inches) < 0.5);
    }

    public static Command maybeScoreCoral(Swerve swerve, Elevator elevator,
        CoralScoring coralScoring, ElevatorAlgae algae,
        Supplier<ScoringLocation.CoralLocation> location, Supplier<ScoringLocation.Height> height) {
        return reefPreAlign(swerve, location).andThen(new ConditionalCommand(Commands
            .waitUntil(() -> coralScoring.getOuttakeBeamBreakStatus()).deadlineFor(swerve.stop()),
            Commands.runOnce(() -> {
            }), () -> !height.get().isAlgae))
            .deadlineFor(coralScoring.runCoralIntake()
                .unless(() -> coralScoring.getOuttakeBeamBreakStatus()))
            .andThen(new ConditionalCommand(
                scoreWithElevator(swerve, elevator, location, height).withTimeout(2.4),
                Commands.runOnce(() -> {
                }), () -> !height.get().isAlgae))
            .andThen(new ConditionalCommand(Commands.runOnce(() -> {
            }), coralScoring.runCoralOuttake().withTimeout(0.4), () -> height.get().isAlgae));
    }

    public static Command maybePickupAlgae(Swerve swerve, Elevator elevator, ElevatorAlgae algae,
        Supplier<Optional<ScoringLocation.Height>> algaeHeight,
        Consumer<ScoringLocation.Height> crossOut) {
        return Commands.runOnce(() -> {
        }); // TODO
    }

    /** Move and score coral or retrieve algae. */
    public static Command autoScore(Swerve swerve, Elevator elevator, CoralScoring coralScoring,
        ElevatorAlgae algae, Supplier<ScoringLocation.CoralLocation> location,
        Supplier<ScoringLocation.Height> height,
        Supplier<Optional<ScoringLocation.Height>> additionalAlgaeHeight,
        Consumer<ScoringLocation.Height> crossOut) {

        return maybeScoreCoral(swerve, elevator, coralScoring, algae, location, height)
            .andThen(maybePickupAlgae(swerve, elevator, algae, () -> {
                ScoringLocation.Height currentHeight = height.get();
                if (currentHeight.isAlgae) {
                    return Optional.of(currentHeight);
                } else {
                    return additionalAlgaeHeight.get();
                }
            }, crossOut));
    }

    private static final Pose2d processorPose =
        new Pose2d(6.342151165008545, 0.5018799304962158, Rotation2d.kCW_90deg);

    /** Do something with algae gotten. */
    // public static Command doSomethingWithAlgae(Swerve swerve, Elevator elevator,
    // ElevatorAlgae algae, Supplier<Character> whatToDo) {
    // return new SelectCommand<>(Map.of('d',
    // Commands.sequence(algae.algaeOuttakeCommand().withTimeout(1.0).asProxy()), 'b',
    // Commands.sequence(scoreInBarge(swerve, elevator, algae)), 'p', Commands.sequence()),
    // whatToDo);
    // }

    private static Command feederAfter(Swerve swerve, CoralScoring scoring) {
        return swerve.run(() -> {
            swerve.setModuleStates(new ChassisSpeeds(-0.1, 0.0, 0.0));
        }).until(scoring.coralAtIntake).withTimeout(1.0).andThen(swerve.stop());
    }

    private static final Pose2d leftFeederAway = new Pose2d(1.5196709632873535, 7.158551216125488,
        Rotation2d.fromRadians(-2.4980917038665034).plus(Rotation2d.kCCW_90deg));
    private static final Pose2d leftFeeder = new Pose2d(1.1899291276931763, 7.560424327850342,
        Rotation2d.fromRadians(-2.4980917038665034).plus(Rotation2d.kCCW_90deg));

    /** Move to left feeder */
    public static Command leftFeeder(Swerve swerve, Elevator elevator, CoralScoring coral) {
        return (Commands.waitSeconds(0.2).andThen(ensureHome(elevator)))
            .alongWith(new MoveAndAvoidReef(swerve, () -> leftFeederAway, () -> {

                return Constants.SwerveTransformPID.MAX_VELOCITY;

            }, true, Units.inchesToMeters(100), 45).until(coral.coralAtIntake)
                .andThen(new MoveToPose(swerve, () -> leftFeeder, () -> {

                    return Constants.SwerveTransformPID.MAX_VELOCITY;

                }, true, Units.inchesToMeters(12), 20).until(coral.coralAtIntake)))
            .andThen(feederAfter(swerve, coral));
    }

    private static final Pose2d leftFeederClose = new Pose2d(0.6334888935089111, 6.746373176574707,
        Rotation2d.fromRadians(-0.9272955650632482));

    /** Move to left feeder close */
    public static Command leftFeederClose(Swerve swerve, Elevator elevator, CoralScoring coral) {
        return (Commands.waitSeconds(0.2).andThen(ensureHome(elevator)))
            .alongWith(new MoveAndAvoidReef(swerve, () -> leftFeederClose, () -> {

                return Constants.SwerveTransformPID.MAX_VELOCITY;

            }, true, Units.inchesToMeters(100), 45).until(coral.coralAtIntake)
                .andThen(new MoveToPose(swerve, () -> leftFeederClose, () -> {

                    return Constants.SwerveTransformPID.MAX_VELOCITY;

                }, true, Units.inchesToMeters(12), 20).until(coral.coralAtIntake)))
            .andThen(feederAfter(swerve, coral));
    }


    private static final Pose2d rightFeederAway = new Pose2d(leftFeederAway.getX(),
        FieldConstants.fieldWidth.in(Meters) - leftFeederAway.getY(),
        leftFeederAway.getRotation().unaryMinus());
    private static final Pose2d rightFeeder =
        new Pose2d(leftFeeder.getX(), FieldConstants.fieldWidth.in(Meters) - leftFeeder.getY(),
            leftFeeder.getRotation().unaryMinus());

    /** Move to right feeder */
    public static Command rightFeeder(Swerve swerve, Elevator elevator, CoralScoring coral) {
        return (Commands.waitSeconds(0.2).andThen(ensureHome(elevator)))
            .alongWith(new MoveAndAvoidReef(swerve, () -> rightFeederAway, () -> {
                if (elevator.hightAboveP0.getAsBoolean()) {
                    return Constants.SwerveTransformPID.MAX_ELEVATOR_UP_VELOCITY;
                } else {
                    return Constants.SwerveTransformPID.MAX_VELOCITY;
                }
            }, true, Units.inchesToMeters(24), 45)
                .andThen(new MoveToPose(swerve, () -> rightFeeder, () -> {
                    if (elevator.hightAboveP0.getAsBoolean()) {
                        return Constants.SwerveTransformPID.MAX_ELEVATOR_UP_VELOCITY;
                    } else {
                        return Constants.SwerveTransformPID.MAX_VELOCITY;
                    }
                }, true, Units.inchesToMeters(12), 20)))
            .andThen(feederAfter(swerve, coral));
    }



    private static final Pose2d rightFeederClose = new Pose2d(leftFeederClose.getX(),
        FieldConstants.fieldWidth.in(Meters) - leftFeederClose.getY(),
        leftFeederClose.getRotation().unaryMinus());

    /** Move to left feeder close */
    public static Command rightFeederClose(Swerve swerve, Elevator elevator, CoralScoring coral) {
        return (Commands.waitSeconds(0.2).andThen(ensureHome(elevator)))
            .alongWith(new MoveAndAvoidReef(swerve, () -> rightFeederClose, () -> {

                return Constants.SwerveTransformPID.MAX_VELOCITY;

            }, true, Units.inchesToMeters(100), 45).until(coral.coralAtIntake)
                .andThen(new MoveToPose(swerve, () -> rightFeederClose, () -> {

                    return Constants.SwerveTransformPID.MAX_VELOCITY;

                }, true, Units.inchesToMeters(12), 20).until(coral.coralAtIntake)))
            .andThen(feederAfter(swerve, coral));
    }

    /** Move to faster feeder */
    public static Command fasterFeeder(Swerve swerve, Elevator elevator, CoralScoring coral) {
        Logger.recordOutput("leftFeeder", leftFeeder);
        Logger.recordOutput("rightFeeder", rightFeeder);

        return (Commands.waitSeconds(0.2).andThen(ensureHome(elevator)))
            .alongWith(new MoveAndAvoidReef(swerve, () -> {
                if ((swerve.state.getGlobalPoseEstimate()
                    .getY() > FieldConstants.fieldWidth.in(Meters) / 2) != AllianceFlipUtil
                        .shouldFlip()) {
                    return leftFeeder;
                } else {
                    return rightFeeder;
                }
            }, () -> {
                if (elevator.hightAboveP0.getAsBoolean()) {
                    return Constants.SwerveTransformPID.MAX_ELEVATOR_UP_VELOCITY;
                } else {
                    return Constants.SwerveTransformPID.MAX_VELOCITY;
                }
            }, true, Units.inchesToMeters(12), 20)).andThen(feederAfter(swerve, coral));
    }

    /** Move to feeder depending on webcontroller code */
    public static Command selectFeeder(Swerve swerve, Elevator elevator, CoralScoring coral,
        Supplier<Character> charSupplier) {
        return (Commands.waitSeconds(0.2).andThen(ensureHome(elevator)))
            .alongWith(new MoveAndAvoidReef(swerve, () -> {
                char target = charSupplier.get();
                if (target == 'f') {
                    if ((swerve.state.getGlobalPoseEstimate()
                        .getY() > FieldConstants.fieldWidth.in(Meters) / 2) != AllianceFlipUtil
                            .shouldFlip()) {
                        return leftFeeder;
                    } else {
                        return rightFeeder;
                    }
                } else if (target == 'r') {
                    return rightFeeder;
                } else {
                    return leftFeeder;
                }
            }, () -> {
                if (elevator.hightAboveP0.getAsBoolean()) {
                    return Constants.SwerveTransformPID.MAX_ELEVATOR_UP_VELOCITY;
                } else {
                    return Constants.SwerveTransformPID.MAX_VELOCITY;
                }
            }, true, Units.inchesToMeters(12), 20)).andThen(feederAfter(swerve, coral));
    }

    // private static final Pose2d bargePose = new Pose2d(FieldConstants.fieldLength.in(Meters) -
    // 9.49,
    // 5.551057815551758, Rotation2d.kZero);

    // /** Move to barge position */
    // public static Command barge(Swerve swerve, Elevator elevator) {
    // return ensureHome(elevator).alongWith(new MoveAndAvoidReef(swerve, () -> bargePose, () -> {
    // if (elevator.hightAboveP0.getAsBoolean()) {
    // return Constants.SwerveTransformPID.MAX_ELEVATOR_UP_VELOCITY;
    // } else {
    // return Constants.SwerveTransformPID.MAX_VELOCITY;
    // }
    // }, true, Units.inchesToMeters(12), 3));
    // }

    private static final Pose2d bargeScorePose = new Pose2d(
        7.558475971221924 + Units.inchesToMeters(9), 6.258963108062744, Rotation2d.kZero);

    /**
     * move and score in barge final
     *
     * @param swerve swerve
     * @param elevator elevator
     * @param algae algae
     * @return move and score in barge
     */
    public static Command scoreInBarge(Swerve swerve, Elevator elevator, ElevatorAlgae algae) {
        return (ensureHome(elevator)
            .alongWith(new MoveAndAvoidReef(swerve, () -> bargeScorePose, () -> {
                if (elevator.hightAboveP0.getAsBoolean()) {
                    return Constants.SwerveTransformPID.MAX_ELEVATOR_UP_VELOCITY;
                } else {
                    return Constants.SwerveTransformPID.MAX_VELOCITY;
                }
            }, true, Units.inchesToMeters(6), 3))).andThen(bargeSpitAlgae(elevator, algae)
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    }

    /**
     * Command to Raise Elevator and spit algae into the barge
     *
     * @param elevator Elevator Subsystem
     * @param algae Algae Subsystem
     * @return Command
     */
    public static Command bargeSpitAlgae(Elevator elevator, ElevatorAlgae algae) {
        return Commands.waitUntil(() -> elevator.getHeight().in(Inches) > 65)
            .deadlineFor(elevator.p5())
            .andThen(algae.algaeOuttakeCommand().withTimeout(.3).asProxy())
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }
}
