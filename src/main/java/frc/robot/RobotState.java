package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import java.util.List;
import java.util.Optional;
import java.util.stream.Stream;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.math.Circle;
import frc.lib.math.Hexagon;
import frc.lib.math.Penetration;
import frc.lib.math.Rectangle;
import frc.lib.math.SeparatingAxis;
import frc.lib.util.viz.Viz2025;
import frc.robot.subsystems.swerve.Swerve;

/** Primary Drivetrain State Estimator */
public class RobotState {

    private final Viz2025 vis;
    private boolean isInitialized = false;

    private final TimeInterpolatableBuffer<Rotation2d> rotationBuffer =
        TimeInterpolatableBuffer.createBuffer(1.5);

    public RobotState(Viz2025 vis) {
        this.vis = vis;
    }

    private SwerveDrivePoseEstimator swerveOdometry = null;

    /**
     * Initialize this {@link RobotState}. Should only be called once (usually from the
     * {@link Swerve} constructor).
     */
    public void init(SwerveModulePosition[] positions, Rotation2d gyroYaw) {
        swerveOdometry = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, gyroYaw,
            positions, new Pose2d(0, 0, Rotation2d.k180deg.minus(Rotation2d.fromDegrees(60))));
        rotationBuffer.clear();
        isInitialized = false;
        SmartDashboard.putNumber("cameraOffset", 0.0);
    }

    private double visionCutoff = 0;

    /**
     * Use prior information to set the pose. Should only be used at the start of the program, or
     * start of individual autonomous routines.
     */
    public void resetPose(Pose2d pose, SwerveModulePosition[] positions, Rotation2d gyroYaw) {
        swerveOdometry.resetPosition(gyroYaw, positions, pose);
        visionCutoff = Timer.getFPGATimestamp();
        rotationBuffer.clear();
        rotationBuffer.getInternalBuffer().clear();
    }

    private Optional<Rotation2d> sampleRotationAt(double timestampSeconds) {
        if (rotationBuffer.getInternalBuffer().isEmpty()) {
            return Optional.empty();
        }

        double oldestOdometryTimestamp = rotationBuffer.getInternalBuffer().firstKey();
        double newestOdometryTimestamp = rotationBuffer.getInternalBuffer().lastKey();
        if (oldestOdometryTimestamp > timestampSeconds) {
            return Optional.empty();
        }
        timestampSeconds =
            MathUtil.clamp(timestampSeconds, oldestOdometryTimestamp, newestOdometryTimestamp);

        return rotationBuffer.getSample(timestampSeconds);
    }

    /**
     * Get the current pose estimate using the global solver.
     */
    public Pose2d getGlobalPoseEstimate() {
        return swerveOdometry.getEstimatedPosition();
    }

    private final Circle stdDevGlobalCircle =
        new Circle("State/GlobalEstimateStdDev", new Translation2d(), 0);
    private final Circle stdDevLocalCircle =
        new Circle("State/LocalEstimateStdDev", new Translation2d(), 0);

    private void addVisionObservation(Pose3d cameraPose, Pose3d robotPose, double timestamp,
        Vector<N3> baseUncertainty, List<PhotonTrackedTarget> targets, String prefix,
        boolean doInit, Circle circle) {
        double totalDistance = 0.0;
        int count = 0;
        for (var tag : targets) {
            var maybeTagPose = Constants.Vision.fieldLayout.getTagPose(tag.getFiducialId());
            if (maybeTagPose.isPresent()) {
                var tagPose = maybeTagPose.get();
                totalDistance += tagPose.getTranslation().getDistance(cameraPose.getTranslation());
                count++;
            }
        }
        double avgDistance = totalDistance / count;
        double stddev = Math.pow(avgDistance, 2.0) / count;
        Pose2d robotPose2d = robotPose.toPose2d();
        if (Constants.shouldDrawStuff) {
            circle.setCenter(robotPose2d.getTranslation());
            circle.setRadius(stddev * baseUncertainty.get(0));
            circle.drawImpl();
            Logger.recordOutput("State/" + prefix + "VisionEstimate", robotPose);
            Logger.recordOutput("State/" + prefix + "AverageDistance", avgDistance);
            Logger.recordOutput("State/" + prefix + "StdDevMultiplier", stddev);
            Logger.recordOutput("State/" + prefix + "XYStdDev", stddev * baseUncertainty.get(0));
            Logger.recordOutput("State/" + prefix + "ThetaStdDev", stddev * baseUncertainty.get(2));
        }
        if (doInit && !isInitialized) {
            swerveOdometry.resetPose(robotPose2d);
            isInitialized = true;
        } else if (isInitialized) {
            swerveOdometry.addVisionMeasurement(robotPose2d, timestamp,
                baseUncertainty.times(stddev));
        }
    }

    /**
     * Add information from cameras.
     */
    public void addVisionObservation(PhotonPipelineResult result, Transform3d robotToCamera,
        int whichCamera) {
        if (result.getTimestampSeconds() < visionCutoff) {
            return;
        }
        if (whichCamera == 0 && result.multitagResult.isPresent()) {
            Transform3d best = result.multitagResult.get().estimatedPose.best;
            Pose3d cameraPose =
                new Pose3d().plus(best).relativeTo(Constants.Vision.fieldLayout.getOrigin());
            Pose3d robotPose = cameraPose.plus(robotToCamera.inverse());
            addVisionObservation(cameraPose, robotPose, result.getTimestampSeconds(),
                VecBuilder.fill(Constants.StateEstimator.globalVisionTrust.getAsDouble(),
                    Constants.StateEstimator.globalVisionTrust.getAsDouble(),
                    Constants.StateEstimator.globalVisionTrustRotation.getAsDouble()),
                result.getTargets(), "Global", true, stdDevGlobalCircle);
        }
        if (whichCamera == 1) {
            for (var target : result.targets) {
                double dist =
                    target.getBestCameraToTarget().getTranslation().toTranslation2d().getNorm();
                if (dist > Units.inchesToMeters(36)) {
                    continue;
                }
                localCircle.setRadius(dist - Constants.Vision.cameras[whichCamera].offset());
                localCircle.setCenter(Constants.Vision.fieldLayout.getTagPose(target.fiducialId)
                    .get().getTranslation().toTranslation2d());
                localCircle.draw();
                Optional<Rotation2d> maybeRobotYaw = sampleRotationAt(result.getTimestampSeconds());
                Rotation2d robotYaw;
                if (maybeRobotYaw.isPresent()) {
                    robotYaw = maybeRobotYaw.get();
                } else {
                    continue;
                }
                Rotation2d yaw = Rotation2d.fromDegrees(robotYaw.getDegrees() - target.getYaw()
                    + 180 + Units.radiansToDegrees(robotToCamera.getRotation().getZ()));
                xCircle.setCenter(localCircle.getVertex(yaw));
                xCircle.draw();
                Pose2d robotPose2d = new Pose2d(
                    xCircle.getCenter()
                        .minus(robotToCamera.getTranslation().toTranslation2d().rotateBy(robotYaw)),
                    robotYaw);
                Pose3d robotPose = new Pose3d(robotPose2d);
                Pose3d cameraPose = robotPose.plus(robotToCamera);
                addVisionObservation(cameraPose, robotPose, result.getTimestampSeconds(),
                    VecBuilder.fill(Constants.StateEstimator.localVisionTrust.getAsDouble(),
                        Constants.StateEstimator.localVisionTrust.getAsDouble(),
                        Double.POSITIVE_INFINITY),
                    result.getTargets(), "Local", false, stdDevLocalCircle);
            }
        }
    }

    private final Circle localCircle =
        new Circle("State/LocalEstimationDistance", new Translation2d(), 0);
    private final Circle xCircle =
        new Circle("State/LocalEstimationPose", new Translation2d(), Units.inchesToMeters(2));

    /**
     * Add information from swerve drive.
     */
    public void addSwerveObservation(SwerveModulePosition[] positions, Rotation2d gyroYaw) {
        swerveOdometry.update(gyroYaw, positions);
        constrain(positions, gyroYaw);
        rotationBuffer.addSample(MathSharedStore.getTimestamp(),
            getGlobalPoseEstimate().getRotation());
        vis.setDrivetrainState(swerveOdometry.getEstimatedPosition(),
            Stream.of(positions).map(x -> x.angle).toArray(this::swerveRotationsArray));
        stdDevGlobalCircle.setCenter(getGlobalPoseEstimate().getTranslation());
        stdDevGlobalCircle.setRadius(stdDevGlobalCircle.getRadius() + 0.01);
        stdDevLocalCircle.setCenter(new Translation2d());
        stdDevLocalCircle.setRadius(0.0);
    }

    private Rotation2d[] swerveRotations = new Rotation2d[4];

    /**
     * To avoid allocating every loop, we create this function to accommodate a
     * {@link Stream.toArray} call.
     */
    private Rotation2d[] swerveRotationsArray(int _count) {
        return swerveRotations;
    }

    private void constrain(SwerveModulePosition[] positions, Rotation2d gyroYaw) {
        var original = getGlobalPoseEstimate();
        double x = original.getX();
        double y = original.getY();
        double t = -original.getRotation().getRadians();

        Translation2d[] bumpers = new Translation2d[5];
        Translation2d[] tr = new Translation2d[4];
        for (int i = 0; i < 4; i++) {
            double theta = t + i * Math.PI / 2;
            tr[i] = new Translation2d(
                x + Math.cos(theta) * Constants.Swerve.bumperFront.in(Meters)
                    + Math.sin(theta) * Constants.Swerve.bumperRight.in(Meters),
                y - Math.sin(theta) * Constants.Swerve.bumperFront.in(Meters)
                    + Math.cos(theta) * Constants.Swerve.bumperRight.in(Meters));
            bumpers[i] = tr[i];
        }
        bumpers[4] = bumpers[0];

        double dx = 0.0;
        double dy = 0.0;

        if (Constants.StateEstimator.keepInField) {

            double maxY = FieldConstants.fieldWidth.in(Meters);
            double maxX = FieldConstants.fieldLength.in(Meters);

            for (int i = 0; i < 4; i++) {
                double x1 = tr[i].getX();
                double y1 = tr[i].getY();
                double x2 = maxX - x1;

                // Simple keep in rect
                if (x1 < 0.0) {
                    if (-x1 > Math.abs(dx)) {
                        dx = -x1;
                    }
                } else if (x1 > maxX) {
                    double mdx = x1 - maxX;
                    if (mdx > Math.abs(dx)) {
                        dx = -mdx;
                    }
                }
                if (y1 < 0.0) {
                    if (-y1 > Math.abs(dy)) {
                        dy = -y1;
                    }
                } else if (y1 > maxY) {
                    double mdy = y1 - maxY;
                    if (mdy > Math.abs(dy)) {
                        dy = -mdy;
                    }
                }

                // Coral Stations

                if (FieldConstants.CoralStation.rightM * x1
                    + FieldConstants.CoralStation.rightB > y1) {
                    double newM = -1.0 / FieldConstants.CoralStation.rightM;
                    double newB = y1 - newM * x1;
                    double newX = (newB - FieldConstants.CoralStation.rightB)
                        / (FieldConstants.CoralStation.rightM - newM);
                    double newY = newM * newX + newB;
                    double newDx = (newX - x1);
                    double newDy = (newY - y1);
                    double newNormSqr = newDx * newDx + newDy * newDy;
                    double oldNormSqr = dx * dx + dy * dy;
                    if (newNormSqr > oldNormSqr) {
                        dx = newDx;
                        dy = newDy;
                    }
                } else if (FieldConstants.CoralStation.leftM * x1
                    + FieldConstants.CoralStation.leftB < y1) {
                    double newM = -1.0 / FieldConstants.CoralStation.leftM;
                    double newB = y1 - newM * x1;
                    double newX = (newB - FieldConstants.CoralStation.leftB)
                        / (FieldConstants.CoralStation.leftM - newM);
                    double newY = newM * newX + newB;
                    double newDx = (newX - x1);
                    double newDy = (newY - y1);
                    double newNormSqr = newDx * newDx + newDy * newDy;
                    double oldNormSqr = dx * dx + dy * dy;
                    if (newNormSqr > oldNormSqr) {
                        dx = newDx;
                        dy = newDy;
                    }
                } else if (FieldConstants.CoralStation.rightM * x2
                    + FieldConstants.CoralStation.rightB > y1) {
                    double newM = -1.0 / FieldConstants.CoralStation.rightM;
                    double newB = y1 - newM * x2;
                    double newX = (newB - FieldConstants.CoralStation.rightB)
                        / (FieldConstants.CoralStation.rightM - newM);
                    double newY = newM * newX + newB;
                    double newDx = (newX - x2);
                    double newDy = (newY - y1);
                    double newNormSqr = newDx * newDx + newDy * newDy;
                    double oldNormSqr = dx * dx + dy * dy;
                    if (newNormSqr > oldNormSqr) {
                        dx = -newDx;
                        dy = newDy;
                    }
                } else if (FieldConstants.CoralStation.leftM * x2
                    + FieldConstants.CoralStation.leftB < y1) {
                    double newM = -1.0 / FieldConstants.CoralStation.leftM;
                    double newB = y1 - newM * x2;
                    double newX = (newB - FieldConstants.CoralStation.leftB)
                        / (FieldConstants.CoralStation.leftM - newM);
                    double newY = newM * newX + newB;
                    double newDx = (newX - x2);
                    double newDy = (newY - y1);
                    double newNormSqr = newDx * newDx + newDy * newDy;
                    double oldNormSqr = dx * dx + dy * dy;
                    if (newNormSqr > oldNormSqr) {
                        dx = -newDx;
                        dy = newDy;
                    }
                }
            }
        }

        // Reef check

        reef1.draw();
        reef2.draw();
        centerPost.draw();
        robot.setPose(original);
        robot.draw();

        if (Constants.StateEstimator.keepOutOfReefs) {
            Penetration penetration = new Penetration("ReefPen");
            Translation2d[] sepResult =
                new Translation2d[] {Translation2d.kZero, Translation2d.kZero};
            if (SeparatingAxis.solve(robot, reef1, penetration)) {
                sepResult[0] = robot.getCenter();
                Translation2d offs = new Translation2d(penetration.getDepth(),
                    new Rotation2d(penetration.getXDir(), penetration.getYDir()));
                Translation2d resPos = robot.getCenter().plus(offs);
                robotTest.setPose(new Pose2d(resPos, original.getRotation()));
                sepResult[1] = robotTest.getCenter();
                penetration.draw();

                dx = offs.getX();
                dy = offs.getY();
            } else if (SeparatingAxis.solve(robot, reef2, penetration)) {
                sepResult[0] = robot.getCenter();
                Translation2d offs = new Translation2d(penetration.getDepth(),
                    new Rotation2d(penetration.getXDir(), penetration.getYDir()));
                Translation2d resPos = robot.getCenter().plus(offs);
                robotTest.setPose(new Pose2d(resPos, original.getRotation()));
                sepResult[1] = robotTest.getCenter();
                penetration.draw();

                dx = offs.getX();
                dy = offs.getY();
            } else if (SeparatingAxis.solve(robot, centerPost, penetration)) {
                sepResult[0] = robot.getCenter();
                Translation2d offs = new Translation2d(penetration.getDepth(),
                    new Rotation2d(penetration.getXDir(), penetration.getYDir()));
                Translation2d resPos = robot.getCenter().plus(offs);
                robotTest.setPose(new Pose2d(resPos, original.getRotation()));
                sepResult[1] = robotTest.getCenter();
                penetration.draw();

                dx = offs.getX();
                dy = offs.getY();
            } else {
                robotTest.setPose(new Pose2d(-100, -100, Rotation2d.kZero));
            }
            robotTest.draw();

            Logger.recordOutput("sepResult", sepResult);
        }

        // TODO

        if (Math.abs(dx) > 0.01 || Math.abs(dy) > 0.01) {
            resetPose(new Pose2d(x + dx, y + dy, original.getRotation()), positions, gyroYaw);
            return;
        }
    }

    private final Hexagon reef1 = new Hexagon("BlueReef", FieldConstants.Reef.center,
        FieldConstants.Reef.circumscribedRadius.in(Meters), Rotation2d.fromDegrees(30));
    private final Hexagon reef2 = new Hexagon("RedReef",
        new Translation2d(FieldConstants.fieldLength.in(Meters) - FieldConstants.Reef.center.getX(),
            FieldConstants.Reef.center.getY()),
        FieldConstants.Reef.circumscribedRadius.in(Meters), Rotation2d.fromDegrees(30));
    private final Rectangle centerPost = new Rectangle("CenterPost",
        new Pose2d(new Translation2d(FieldConstants.fieldLength.in(Meters) / 2,
            FieldConstants.fieldWidth.in(Meters) / 2), Rotation2d.kZero),
        0.4, 0.4);
    private final Rectangle robot = new Rectangle("Robot", new Pose2d(),
        Constants.Swerve.bumperFront.in(Meters) * 2, Constants.Swerve.bumperRight.in(Meters) * 2);
    private final Rectangle robotTest = new Rectangle("RobotTest", new Pose2d(),
        Constants.Swerve.bumperFront.in(Meters) * 2, Constants.Swerve.bumperRight.in(Meters) * 2);

}
