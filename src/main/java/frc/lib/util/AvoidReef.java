package frc.lib.util;

import static edu.wpi.first.units.Units.Meters;
import java.util.ArrayList;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.math.AllianceFlipUtil;
import frc.lib.math.Circle;
import frc.lib.math.RotationInterval;
import frc.lib.util.viz.Drawable;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.swerve.Swerve;

public class AvoidReef extends Command implements Drawable {

    private static final double driveCircleRadius =
        FieldConstants.Reef.circumscribedRadius.in(Meters) + Math.hypot(
            Constants.Swerve.bumperFront.in(Meters), Constants.Swerve.bumperRight.in(Meters));
    private static final Circle redDriveCircle = new Circle("avoidReef/redDriveCircle",
        new Translation2d(FieldConstants.fieldLength.in(Meters) - FieldConstants.Reef.center.getX(),
            FieldConstants.Reef.center.getY()),
        driveCircleRadius);
    private static final Circle blueDriveCircle =
        new Circle("avoidReef/blueDriveCircle", FieldConstants.Reef.center, driveCircleRadius);

    private static Pose2d avoidReef(Circle driveCircle, Translation2d currentPosition,
        Translation2d targetPosition, double overlap, RotationInterval currentAngles,
        RotationInterval targetAngles) {
        if (overlap < 1e-6) {
            Translation2d targetMax = driveCircle.getVertex(targetAngles.getMax());
            Translation2d targetMin = driveCircle.getVertex(targetAngles.getMin());
            Translation2d currentMax = driveCircle.getVertex(currentAngles.getMax());
            Translation2d currentMin = driveCircle.getVertex(currentAngles.getMin());

            double rightLen =
                RotationInterval.acute(targetAngles.getMax(), currentAngles.getMin()).range()
                    * driveCircleRadius + currentPosition.minus(currentMin).getNorm()
                    + targetPosition.minus(targetMax).getNorm();
            double leftLen =
                RotationInterval.acute(targetAngles.getMin(), currentAngles.getMax()).range()
                    * driveCircleRadius + currentPosition.minus(currentMax).getNorm()
                    + targetPosition.minus(targetMin).getNorm();
            if (driveCircle.sdf(currentPosition) < Units.inchesToMeters(10)) {
                Rotation2d angle = driveCircle.getAngle(currentPosition);
                if (rightLen > leftLen) {
                    Rotation2d targetAngle =
                        angle.plus(Constants.SwerveTransformPID.CIRCLE_REEF_LOOKAHEAD_ANGLE);
                    return new Pose2d(driveCircle.getVertex(targetAngle, Units.inchesToMeters(3)),
                        targetAngle.plus(Rotation2d.k180deg));
                } else {
                    Rotation2d targetAngle =
                        angle.minus(Constants.SwerveTransformPID.CIRCLE_REEF_LOOKAHEAD_ANGLE);
                    return new Pose2d(driveCircle.getVertex(targetAngle, Units.inchesToMeters(3)),
                        targetAngle.plus(Rotation2d.k180deg));
                }
            } else {
                if (rightLen > leftLen) {
                    return new Pose2d(currentMax, currentAngles.getMax().plus(Rotation2d.k180deg));
                } else {
                    return new Pose2d(currentMin, currentAngles.getMin().plus(Rotation2d.k180deg));
                }
            }
        }
        return null;
    }

    private static Pose2d getNextIntermediateTarget(Pose2d current, Pose2d target) {
        Translation2d currentPosition = current.getTranslation();
        Translation2d targetPosition = target.getTranslation();

        RotationInterval targetBlueAngles =
            blueDriveCircle.circleTangentAngles(target.getTranslation());
        RotationInterval targetRedAngles =
            redDriveCircle.circleTangentAngles(target.getTranslation());
        RotationInterval currentBlueAngles = blueDriveCircle.circleTangentAngles(currentPosition);
        RotationInterval currentRedAngles = redDriveCircle.circleTangentAngles(currentPosition);

        double blueOverlap = targetBlueAngles.getOverlap(currentBlueAngles);
        double redOverlap = targetRedAngles.getOverlap(currentRedAngles);

        Logger.recordOutput("avoidReef/blueOverlap", blueOverlap);
        Logger.recordOutput("avoidReef/redOverlap", redOverlap);

        if (currentPosition.getX() > FieldConstants.fieldLength.in(Meters) / 2) {
            // On red side, check red first, then blue.
            Pose2d next = null;
            if ((next = avoidReef(redDriveCircle, currentPosition, targetPosition, redOverlap,
                currentRedAngles, targetRedAngles)) != null) {
                return next;
            }
            if ((next = avoidReef(blueDriveCircle, currentPosition, targetPosition, blueOverlap,
                currentBlueAngles, targetBlueAngles)) != null) {
                return next;
            }
        } else {
            // On blue side, check blue first, then red.
            Pose2d next = null;
            if ((next = avoidReef(blueDriveCircle, currentPosition, targetPosition, blueOverlap,
                currentBlueAngles, targetBlueAngles)) != null) {
                return next;
            }
            if ((next = avoidReef(redDriveCircle, currentPosition, targetPosition, redOverlap,
                currentRedAngles, targetRedAngles)) != null) {
                return next;
            }
        }

        return target;
    }

    private static boolean isDone(Pose2d current, Pose2d target, double tol, double rotTol) {
        Pose2d poseError = Pose2d.kZero.plus(target.minus(current));
        final var eTranslate = poseError.getTranslation();
        final var eRotate = poseError.getRotation();
        double xyErr = Math.hypot(eTranslate.getX(), eTranslate.getY());
        Logger.recordOutput("avoidReef/xyErr", tol - xyErr);
        Logger.recordOutput("avoidReef/tErr", rotTol - Math.abs(eRotate.getDegrees()));
        return xyErr < tol && Math.abs(eRotate.getDegrees()) < rotTol;
    }

    private final Supplier<Pose2d> currentPose;
    private final Consumer<Pose2d> intermediateTarget;
    private final Supplier<Pose2d> finalTarget;
    private Pose2d targetPose;

    private final double tol;
    private final double rotTol;

    private final boolean flipForRed;

    public AvoidReef(Swerve swerve, boolean flipForRed, Supplier<Pose2d> currentPose,
        Consumer<Pose2d> intermediateTarget, Supplier<Pose2d> finalTarget, double tol,
        double rotTol) {
        this.flipForRed = flipForRed;
        this.currentPose = currentPose;
        this.intermediateTarget = intermediateTarget;
        this.finalTarget = finalTarget;
        this.tol = tol;
        this.rotTol = rotTol;
        addRequirements(swerve);
    }

    @Override
    public void draw() {
        Pose2d current = currentPose.get();
        ArrayList<Pose2d> poses = new ArrayList<>();
        poses.add(current);
        for (int i = 0; i < 500; i++) {
            if (isDone(current, targetPose, tol, rotTol)) {
                break;
            }
            Pose2d next = getNextIntermediateTarget(current, targetPose);
            Transform2d nextDisc = next.minus(current);
            double nextLen = nextDisc.getTranslation().getNorm();
            if (nextLen <= 1e-6) {
                break;
            }
            Transform2d transform =
                new Transform2d(new Translation2d(Math.max(nextLen, Units.inchesToMeters(4)),
                    nextDisc.getTranslation().getAngle()), nextDisc.getRotation());
            next = current.plus(transform);
            poses.add(next);
            current = next;
        }
        Logger.recordOutput("avoidReef/path", poses.toArray(Pose2d[]::new));
        redDriveCircle.draw();
        blueDriveCircle.draw();
    }

    @Override
    public void execute() {
        draw();
        intermediateTarget.accept(getNextIntermediateTarget(currentPose.get(), targetPose));
    }

    @Override
    public void initialize() {
        targetPose = finalTarget.get();
        if (flipForRed) {
            targetPose = AllianceFlipUtil.apply(targetPose);
        }
    }

    @Override
    public boolean isFinished() {
        return isDone(currentPose.get(), targetPose, tol, rotTol);
    }

}
