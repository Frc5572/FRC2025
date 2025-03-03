package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import java.util.Optional;
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
import frc.lib.util.AllianceFlipUtil;
import frc.lib.util.Container;
import frc.lib.util.ScoringLocation;
import frc.robot.commands.MoveAndAvoidReef;
import frc.robot.commands.MoveToPose;
import frc.robot.subsystems.coral.CoralScoring;
import frc.robot.subsystems.elevator.Elevator;
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
        }, () -> 5.0, false, Units.inchesToMeters(2), 10).andThen(swerve.stop());
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
        }, () -> 4.0, true, Units.inchesToMeters(12), 2);
    }

    /** Align to a given scoring location, assuming elevator is at the right height. */
    public static Command reefAlign(Swerve swerve,
        Supplier<ScoringLocation.CoralLocation> location) {
        return new MoveToPose(swerve, () -> {
            Pose2d finalLoc = location.get().pose;

            return new Pose2d(
                finalLoc.getTranslation()
                    .minus(new Translation2d(Units.inchesToMeters(1.0), finalLoc.getRotation())),
                finalLoc.getRotation());
        }, () -> 0.3, true, Units.inchesToMeters(0.25), 0.2);
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
        }, () -> 0.3, true, Units.inchesToMeters(4), 5).withTimeout(1.5);
    }

    /** Go home, no exception */
    public static Command ensureHome(Elevator elevator) {
        return elevator.home().repeatedly().until(() -> elevator.getHeight().in(Inches) < 0.5);
    }

    /** Move and score coral or retrieve algae. */
    public static Command autoScore(Swerve swerve, Elevator elevator, CoralScoring coralScoring,
        Supplier<ScoringLocation.CoralLocation> location, Supplier<ScoringLocation.Height> height,
        Supplier<Optional<ScoringLocation.Height>> additionalAlgaeHeight,
        Container<Boolean> intakingAlgae) {

        return (reefPreAlign(swerve, location).andThen(new ConditionalCommand(
            Commands.waitUntil(coralScoring.coralAtOuttake), Commands.runOnce(() -> {
            }), () -> !height.get().isAlgae))).alongWith(coralScoring.runCoralIntake())

                .andThen(new ConditionalCommand(Commands.runOnce(() -> {
                    intakingAlgae.value = true;
                }), Commands.runOnce(() -> {
                }), () -> height.get().isAlgae || additionalAlgaeHeight.get().isPresent()))
                .andThen(new ConditionalCommand(elevator
                    .moveTo(() -> additionalAlgaeHeight.get().get().height)
                    .andThen(reefAlign(swerve, location)).andThen(backAwayReef(swerve, location)),
                    Commands.runOnce(() -> {
                    }), () -> {
                        return additionalAlgaeHeight.get().isPresent();
                    }))
                .andThen(
                    elevator.moveTo(() -> height.get().height).andThen(reefAlign(swerve, location)))
                .andThen(new ConditionalCommand(Commands.runOnce(() -> {
                }), coralScoring.runCoralOuttake().withTimeout(0.4), () -> height.get().isAlgae))
                .andThen(backAwayReef(swerve, location));
    }

    private static Command feederAfter(Swerve swerve, CoralScoring scoring) {
        return swerve.run(() -> {
            swerve.setModuleStates(new ChassisSpeeds(0.0, -0.1, 0.0));
        }).until(scoring.coralAtIntake).withTimeout(3.0).andThen(swerve.stop());
    }

    private static final Pose2d leftFeeder = new Pose2d(1.5196709632873535, 7.158551216125488,
        Rotation2d.fromRadians(-2.4980917038665034));

    /** Move to left feeder */
    public static Command leftFeeder(Swerve swerve, Elevator elevator, CoralScoring coral) {
        return ensureHome(elevator).alongWith(new MoveAndAvoidReef(swerve, () -> leftFeeder, () -> {
            if (elevator.hightAboveP0.getAsBoolean()) {
                return 0.8;
            } else {
                return 4.0;
            }
        }, true, Units.inchesToMeters(12), 20)).andThen(feederAfter(swerve, coral));
    }

    private static final Pose2d rightFeeder =
        new Pose2d(leftFeeder.getX(), FieldConstants.fieldWidth.in(Meters) - leftFeeder.getY(),
            leftFeeder.getRotation().unaryMinus().plus(Rotation2d.k180deg));

    /** Move to right feeder */
    public static Command rightFeeder(Swerve swerve, Elevator elevator, CoralScoring coral) {
        return ensureHome(elevator)
            .alongWith(new MoveAndAvoidReef(swerve, () -> rightFeeder, () -> {
                if (elevator.hightAboveP0.getAsBoolean()) {
                    return 0.8;
                } else {
                    return 4.0;
                }
            }, true, Units.inchesToMeters(12), 20)).andThen(feederAfter(swerve, coral));
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
                return 0.8;
            } else {
                return 12.0;
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
                return 0.8;
            } else {
                return 12.0;
            }
        }, true, Units.inchesToMeters(12), 20)).andThen(feederAfter(swerve, coral));
    }

    private static final Pose2d bargePose =
        new Pose2d(7.596248722076416, 5.551057815551758, Rotation2d.kZero);

    /** Move to barge position */
    public static Command barge(Swerve swerve, Elevator elevator) {
        return ensureHome(elevator).alongWith(new MoveAndAvoidReef(swerve, () -> bargePose, () -> {
            if (elevator.hightAboveP0.getAsBoolean()) {
                return 0.8;
            } else {
                return 12.0;
            }
        }, true, Units.inchesToMeters(12), 3));
    }

}
