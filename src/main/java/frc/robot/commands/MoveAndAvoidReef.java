package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import java.util.ArrayList;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import choreo.auto.AutoRoutine;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.math.Circle;
import frc.lib.math.RotationInterval;
import frc.lib.util.AllianceFlipUtil;
import frc.lib.util.viz.Drawable;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.swerve.Swerve;

/**
 * Move to Pose2d
 */
public class MoveAndAvoidReef extends Command implements Drawable {

    private EventLoop eventLoop = CommandScheduler.getInstance().getDefaultButtonLoop();
    private AutoRoutine autoRoutine;
    private final Swerve swerve;
    private final Supplier<Pose2d> pose2dSupplier;
    private final DoubleSupplier maxSpeedSupplier;
    private Pose2d pose2d;
    private final boolean flipForRed;
    private final double tol;
    private final double rotTol;
    /** If this trajectory us currently running */
    private boolean isActive = false;
    /** If the trajectory ran to completion */
    private boolean isCompleted = false;

    /**
     * Move to a specified Pose2d command
     *
     * @param swerve Swerve Subsystem
     * @param pose2dSupplier Pose2d Supplier
     * @param maxSpeedSupplier maximum speed to move at
     * @param flipForRed Whether to flip the pose2d for red alliance
     * @param tol Translational Tolerance
     * @param rotTol Rotational Tolerance
     */
    public MoveAndAvoidReef(Swerve swerve, Supplier<Pose2d> pose2dSupplier,
        DoubleSupplier maxSpeedSupplier, boolean flipForRed, double tol, double rotTol) {
        this.swerve = swerve;
        this.pose2dSupplier = pose2dSupplier;
        this.maxSpeedSupplier = maxSpeedSupplier;
        this.flipForRed = flipForRed;
        this.tol = tol;
        this.rotTol = rotTol;
        addRequirements(swerve);
    }

    /**
     * Move to a specified Pose2d command
     *
     * @param swerve Swerve Subsystem
     * @param pose2dSupplier Pose2d Supplier
     * @param maxSpeedSupplier maximum speed to move at
     * @param flipForRed Whether to flip the pose2d for red alliance
     * @param tol Translational Tolerance
     * @param rotTol Rotational Tolerance
     * @param autoRoutine Choreo AutoRoutine to integrate command
     */
    public MoveAndAvoidReef(Swerve swerve, Supplier<Pose2d> pose2dSupplier,
        DoubleSupplier maxSpeedSupplier, boolean flipForRed, double tol, double rotTol,
        AutoRoutine autoRoutine) {
        this(swerve, pose2dSupplier, maxSpeedSupplier, flipForRed, tol, rotTol);
        this.autoRoutine = autoRoutine;
        this.eventLoop = autoRoutine.loop();
    }

    /**
     * Move to a specified Pose2d command
     *
     * @param swerve Swerve Subsystem
     * @param pose2dSupplier Pose2d Supplier
     * @param flipForRed Whether to flip the pose2d for red alliance
     * @param tol Translational Tolerance
     * @param rotTol Rotational Tolerance
     * @param autoRoutine Choreo AutoRoutine to integrate command
     */
    public MoveAndAvoidReef(Swerve swerve, Supplier<Pose2d> pose2dSupplier, boolean flipForRed,
        double tol, double rotTol, AutoRoutine autoRoutine) {
        this(swerve, pose2dSupplier, () -> Constants.Swerve.maxSpeed, flipForRed, tol, rotTol);
        this.autoRoutine = autoRoutine;
        this.eventLoop = autoRoutine.loop();
    }

    /**
     * Returns a trigger that is true while the trajectory is scheduled.
     *
     * @return A trigger that is true while the trajectory is scheduled.
     */
    public Trigger active() {
        if (autoRoutine != null) {
            return new Trigger(eventLoop,
                () -> this.isActive && autoRoutine.active().getAsBoolean());
        }
        return new Trigger(eventLoop, () -> this.isActive);
    }

    public Trigger done() {
        return new Trigger(eventLoop, () -> isCompleted);
    }

    @Override
    public void initialize() {
        isActive = true;
        isCompleted = false;
        pose2d = pose2dSupplier.get();
        if (flipForRed) {
            pose2d = AllianceFlipUtil.apply(pose2d);
        }
    }

    @Override
    public void execute() {
        draw();
        var intermediatePose =
            getNextIntermediateTarget(swerve.state.getGlobalPoseEstimate(), pose2d);
        double distFromTarget =
            intermediatePose.getTranslation().getDistance(pose2d.getTranslation());
        swerve.moveToPose(intermediatePose, maxSpeedSupplier.getAsDouble(),
            Constants.SwerveTransformPID.MAX_ACCELERATION,
            MathUtil.clamp(distFromTarget, 0.5, 1.0) * maxSpeedSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setMotorsZero();
        isActive = false;
        isCompleted = !interrupted;
    }

    @Override
    public boolean isFinished() {
        return isDone(swerve.state.getGlobalPoseEstimate(), pose2d, tol, rotTol);
    }

    public static final double driveCircleRadius =
        FieldConstants.Reef.circumscribedRadius.in(Meters) + Math.hypot(
            1.2 * Constants.Swerve.bumperFront.in(Meters), Constants.Swerve.bumperRight.in(Meters));
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
            if (driveCircle.sdf(currentPosition) < Units.inchesToMeters(20)) {
                Rotation2d angle = driveCircle.getAngle(currentPosition);
                if (rightLen > leftLen) {
                    Rotation2d targetAngle = angle.plus(Constants.CIRCLE_REEF_LOOKAHEAD_ANGLE);
                    return new Pose2d(driveCircle.getVertex(targetAngle, Units.inchesToMeters(3)),
                        targetAngle.plus(Rotation2d.k180deg));
                } else {
                    Rotation2d targetAngle = angle.minus(Constants.CIRCLE_REEF_LOOKAHEAD_ANGLE);
                    return new Pose2d(driveCircle.getVertex(targetAngle, Units.inchesToMeters(3)),
                        targetAngle.plus(Rotation2d.k180deg));
                }
            } else {
                if (rightLen > leftLen) {
                    return new Pose2d(currentMax,
                        driveCircle.getCenter().minus(currentPosition).getAngle());
                } else {
                    return new Pose2d(currentMin,
                        driveCircle.getCenter().minus(currentPosition).getAngle());
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

    @Override
    public void drawImpl() {
        Pose2d current = swerve.state.getGlobalPoseEstimate();
        ArrayList<Pose2d> poses = new ArrayList<>();
        poses.add(current);
        for (int i = 0; i < 500; i++) {
            if (isDone(current, pose2d, tol, rotTol)) {
                break;
            }
            Pose2d next = getNextIntermediateTarget(current, pose2d);
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

    private static boolean isDone(Pose2d current, Pose2d target, double tol, double rotTol) {
        Pose2d poseError = Pose2d.kZero.plus(target.minus(current));
        final var eTranslate = poseError.getTranslation();
        final var eRotate = poseError.getRotation();
        double xyErr = Math.hypot(eTranslate.getX(), eTranslate.getY());
        Logger.recordOutput("avoidReef/xyErr", tol - xyErr);
        Logger.recordOutput("avoidReef/tErr", rotTol - Math.abs(eRotate.getDegrees()));
        return xyErr < tol && Math.abs(eRotate.getDegrees()) < rotTol;
    }
}
