package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import java.util.Map;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.lib.util.AllianceFlipUtil;
import frc.lib.util.Container;
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
        return new MoveToPose(swerve, () -> {
            return swerve.getPose()
                .plus(new Transform2d(Units.inchesToMeters(10), 0, Rotation2d.kZero));
        }, () -> Constants.SwerveTransformPID.MAX_VELOCITY, false, Units.inchesToMeters(2), 10)
            .andThen(swerve.stop());
    }

    /** Approach reef scoring location. Elevator can be home during this. */
    public static Command reefPreAlign(Swerve swerve,
        Supplier<ScoringLocation.CoralLocation> location) {
        return new MoveAndAvoidReef(swerve, () -> {
            Pose2d finalLoc = location.get().pose;
            return new Pose2d(
                finalLoc.getTranslation()
                    .minus(new Translation2d(Units.inchesToMeters(12), finalLoc.getRotation())),
                finalLoc.getRotation().plus(Rotation2d.fromDegrees(15)));
        }, () -> Constants.SwerveTransformPID.MAX_VELOCITY, true, Units.inchesToMeters(12), 5);
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

    /** Go home, no exception */
    public static Command ensureHome(Elevator elevator) {
        return elevator.home().repeatedly().until(() -> elevator.getHeight().in(Inches) < 0.5);
    }

    /** Move and score coral or retrieve algae. */
    public static Command autoScore(Swerve swerve, Elevator elevator, CoralScoring coralScoring,
        ElevatorAlgae algae, Supplier<ScoringLocation.CoralLocation> location,
        Supplier<ScoringLocation.Height> height,
        Supplier<Optional<ScoringLocation.Height>> additionalAlgaeHeight,
        Container<Boolean> intakingAlgae, Consumer<ScoringLocation.Height> crossOut) {

        final Container<ScoringLocation.Height> additionalAlgae =
            new Container<>(ScoringLocation.Height.KP0);

        return (reefPreAlign(swerve, location).andThen(new ConditionalCommand(
            Commands.waitUntil(coralScoring.coralAtOuttake), Commands.runOnce(() -> {
            }), () -> !height.get().isAlgae)))
                .alongWith(coralScoring.runCoralIntake().unless(coralScoring.coralAtOuttake))
                .andThen(new ConditionalCommand(Commands.runOnce(() -> {
                    intakingAlgae.value = true;
                }), Commands.runOnce(() -> {
                }), () -> height.get().isAlgae || additionalAlgaeHeight.get().isPresent()))
                .andThen(new ConditionalCommand(elevator.moveTo(() -> additionalAlgae.value.height)
                    .alongWith(
                        reefAlign(swerve, location, -3).until(algae.hasAlgae).withTimeout(1.2))
                    .alongWith(Commands.runOnce(() -> {
                        crossOut.accept(additionalAlgae.value);
                    })).andThen(backAwayReef(swerve, location).withTimeout(2.0)),
                    Commands.runOnce(() -> {
                    }), () -> {
                        var value = additionalAlgaeHeight.get();
                        if (value.isPresent()) {
                            additionalAlgae.value = value.get();
                            return true;
                        } else {
                            return false;
                        }
                    }))
                .andThen(new ConditionalCommand(
                    (elevator.moveTo(() -> height.get().height)
                        .andThen(reefAlign(swerve, location, 1).withTimeout(2.0))),
                    (elevator.moveTo(() -> height.get().height)
                        .alongWith(reefAlign(swerve, location, 1).withTimeout(2.0))),
                    () -> intakingAlgae.value))
                .andThen(Commands.waitSeconds(0.2))
                .andThen(new ConditionalCommand(Commands.runOnce(() -> {
                }), coralScoring.runCoralOuttake().withTimeout(0.4), () -> height.get().isAlgae))
                .andThen(backAwayReef(swerve, location).withTimeout(2.0));
    }

    private static final Pose2d processorPose =
        new Pose2d(6.342151165008545, 0.5018799304962158, Rotation2d.kCW_90deg);

    /** Do something with algae gotten. */
    public static Command doSomethingWithAlgae(Swerve swerve, Elevator elevator,
        Container<Boolean> intakingAlgae, ElevatorAlgae algae, Supplier<Character> whatToDo,
        DoubleSupplier leftRight) {
        return new ConditionalCommand(
            new SelectCommand<>(Map.of('d', Commands.sequence(Commands.runOnce(() -> {
                intakingAlgae.value = false;
            })), 'b',
                Commands
                    .sequence(ensureHome(elevator).alongWith(new MoveAndAvoidReef(swerve,
                        () -> new Pose2d(bargePose.getX(),
                            bargePose.getY() + leftRight.getAsDouble(), bargePose.getRotation()),
                        () -> {
                            if (elevator.hightAboveP0.getAsBoolean()) {
                                return Constants.SwerveTransformPID.MAX_ELEVATOR_UP_VELOCITY;
                            } else {
                                return Constants.SwerveTransformPID.MAX_VELOCITY;
                            }
                        }, true, Units.inchesToMeters(4), 5)),
                        elevator.moveTo(() -> ScoringLocation.Height.KP5.height),
                        Commands.runOnce(() -> {
                            intakingAlgae.value = false;
                        }),
                        algae.runAlgaeMotor(Constants.Algae.NEGATIVE_VOLTAGE).withTimeout(0.4)
                            .asProxy()),
                'p', Commands
                    .sequence(ensureHome(elevator).alongWith(new MoveAndAvoidReef(swerve, () -> {
                        Pose2d desiredPose = processorPose;
                        return new Pose2d(
                            desiredPose.getTranslation().minus(new Translation2d(
                                Units.inchesToMeters(12), desiredPose.getRotation())),
                            desiredPose.getRotation());
                    }, () -> {
                        if (elevator.hightAboveP0.getAsBoolean()) {
                            return Constants.SwerveTransformPID.MAX_ELEVATOR_UP_VELOCITY;
                        } else {
                            return Constants.SwerveTransformPID.MAX_VELOCITY;
                        }
                    }, true, Units.inchesToMeters(12), 5)),
                        new MoveToPose(swerve, () -> processorPose,
                            () -> Constants.SwerveTransformPID.MAX_ELEVATOR_UP_VELOCITY, true,
                            Units.inchesToMeters(4), 5),
                        Commands.runOnce(() -> {
                            intakingAlgae.value = false;
                        }), algae.runAlgaeMotor(Constants.Algae.NEGATIVE_VOLTAGE).withTimeout(0.4)
                            .asProxy())),
                whatToDo),
            Commands.runOnce(() -> {
            }), () -> intakingAlgae.value);
    }

    private static Command feederAfter(Swerve swerve, CoralScoring scoring) {
        return swerve.run(() -> {
            swerve.setModuleStates(new ChassisSpeeds(0.0, -0.4, 0.0));
        }).until(scoring.coralAtIntake).withTimeout(3.0).andThen(swerve.stop());
    }

    private static final Pose2d leftFeederAway = new Pose2d(1.5196709632873535, 7.158551216125488,
        Rotation2d.fromRadians(-2.4980917038665034));
    private static final Pose2d leftFeeder = new Pose2d(1.4372355937957764, 7.302813529968262,
        Rotation2d.fromRadians(-2.4980917038665034));

    /** Move to left feeder */
    public static Command leftFeeder(Swerve swerve, Elevator elevator, CoralScoring coral) {
        return ensureHome(elevator)
            .alongWith(new MoveAndAvoidReef(swerve, () -> leftFeederAway, () -> {
                if (elevator.hightAboveP0.getAsBoolean()) {
                    return Constants.SwerveTransformPID.MAX_ELEVATOR_UP_VELOCITY;
                } else {
                    return Constants.SwerveTransformPID.MAX_VELOCITY;
                }
            }, true, Units.inchesToMeters(24), 45)
                .andThen(new MoveToPose(swerve, () -> leftFeeder, () -> {
                    if (elevator.hightAboveP0.getAsBoolean()) {
                        return Constants.SwerveTransformPID.MAX_ELEVATOR_UP_VELOCITY;
                    } else {
                        return Constants.SwerveTransformPID.MAX_VELOCITY;
                    }
                }, true, Units.inchesToMeters(12), 20)))
            .andThen(feederAfter(swerve, coral));
    }


    private static final Pose2d rightFeederAway = new Pose2d(leftFeederAway.getX(),
        FieldConstants.fieldWidth.in(Meters) - leftFeederAway.getY(),
        leftFeederAway.getRotation().unaryMinus().plus(Rotation2d.k180deg));
    private static final Pose2d rightFeeder =
        new Pose2d(leftFeeder.getX(), FieldConstants.fieldWidth.in(Meters) - leftFeeder.getY(),
            leftFeeder.getRotation().unaryMinus().plus(Rotation2d.k180deg));

    /** Move to right feeder */
    public static Command rightFeeder(Swerve swerve, Elevator elevator, CoralScoring coral) {
        return ensureHome(elevator)
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

    /** Move to faster feeder */
    public static Command fasterFeeder(Swerve swerve, Elevator elevator, CoralScoring coral) {
        Logger.recordOutput("leftFeeder", leftFeeder);
        Logger.recordOutput("rightFeeder", rightFeeder);

        return ensureHome(elevator).alongWith(new MoveAndAvoidReef(swerve, () -> {
            if ((swerve.state.getGlobalPoseEstimate().getY() > FieldConstants.fieldWidth.in(Meters)
                / 2) != AllianceFlipUtil.shouldFlip()) {
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
        return ensureHome(elevator).alongWith(new MoveAndAvoidReef(swerve, () -> {
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

    private static final Pose2d bargePose = new Pose2d(FieldConstants.fieldLength.in(Meters) - 9.49,
        5.551057815551758, Rotation2d.kZero);

    /** Move to barge position */
    public static Command barge(Swerve swerve, Elevator elevator) {
        return ensureHome(elevator).alongWith(new MoveAndAvoidReef(swerve, () -> bargePose, () -> {
            if (elevator.hightAboveP0.getAsBoolean()) {
                return Constants.SwerveTransformPID.MAX_ELEVATOR_UP_VELOCITY;
            } else {
                return Constants.SwerveTransformPID.MAX_VELOCITY;
            }
        }, true, Units.inchesToMeters(12), 3));
    }

}
