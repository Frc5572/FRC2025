package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import java.util.function.Consumer;
import java.util.stream.Stream;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.lib.math.Hexagon;
import frc.lib.math.Penetration;
import frc.lib.math.Rectangle;
import frc.lib.math.SeparatingAxis;
import frc.lib.util.viz.Viz2025;
import frc.robot.subsystems.swerve.Swerve;

/** Primary Drivetrain State Estimator */
public class RobotState {

    private final Viz2025 vis;

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
            positions, new Pose2d());
        swerveOdometry.setVisionMeasurementStdDevs(VecBuilder.fill(
            Constants.StateEstimator.visionTrust, Constants.StateEstimator.visionTrust,
            Constants.StateEstimator.visionTrustRotation));
    }

    /**
     * Use prior information to set the pose. Should only be used at the start of the program, or
     * start of individual autonomous routines.
     */
    public void resetPose(Pose2d pose, SwerveModulePosition[] positions, Rotation2d gyroYaw) {
        swerveOdometry.resetPosition(gyroYaw, positions, pose);
    }

    /**
     * Get the current pose estimate using the global solver.
     */
    public Pose2d getGlobalPoseEstimate() {
        return swerveOdometry.getEstimatedPosition();
    }

    /**
     * Add information from cameras.
     */
    public void addVisionObservation(PhotonPipelineResult result, Transform3d robotToCamera,
        int whichCamera) {
        if (result.multitagResult.isPresent()) {
            Transform3d best = result.multitagResult.get().estimatedPose.best;
            Pose3d cameraPose =
                new Pose3d().plus(best).relativeTo(Constants.Vision.fieldLayout.getOrigin());
            Pose3d robotPose = cameraPose.plus(robotToCamera.inverse());
            Pose2d robotPose2d = robotPose.toPose2d();
            Logger.recordOutput("State/GlobalVisionEstimate", robotPose);
            swerveOdometry.addVisionMeasurement(robotPose2d, result.getTimestampSeconds());
        }
    }

    /**
     * Add information from swerve drive.
     */
    public void addSwerveObservation(SwerveModulePosition[] positions, Rotation2d gyroYaw) {
        swerveOdometry.update(gyroYaw, positions);
        constrain(getGlobalPoseEstimate(), (newPose) -> resetPose(newPose, positions, gyroYaw));
        vis.setDrivetrainState(swerveOdometry.getEstimatedPosition(),
            Stream.of(positions).map(x -> x.angle).toArray(this::swerveRotationsArray));
    }

    private Rotation2d[] swerveRotations = new Rotation2d[4];

    /**
     * To avoid allocating every loop, we create this function to accommodate a
     * {@link Stream.toArray} call.
     */
    private Rotation2d[] swerveRotationsArray(int _count) {
        return swerveRotations;
    }

    public static Pose2d constrain(Pose2d original, Consumer<Pose2d> reset) {
        double x = original.getX();
        double y = original.getY();
        double t = -original.getRotation().getRadians();

        Translation2d[] tr = new Translation2d[4];
        for (int i = 0; i < 4; i++) {
            double theta = t + i * Math.PI / 2;
            tr[i] = new Translation2d(
                x + Math.cos(theta) * Constants.Swerve.bumperFront.in(Meters)
                    + Math.sin(theta) * Constants.Swerve.bumperRight.in(Meters),
                y - Math.sin(theta) * Constants.Swerve.bumperFront.in(Meters)
                    + Math.cos(theta) * Constants.Swerve.bumperRight.in(Meters));
        }

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
        robotBumperOutline.setPose(original);
        robotBumperOutline.draw();

        if (Constants.StateEstimator.keepOutOfReefs) {
            Penetration penetration = new Penetration("ReefPen");
            Translation2d[] sepResult =
                new Translation2d[] {Translation2d.kZero, Translation2d.kZero};
            if (SeparatingAxis.solve(robotBumperOutline, reef1, penetration)) {
                sepResult[0] = robotBumperOutline.getCenter();
                Translation2d offs = new Translation2d(penetration.getDepth(),
                    new Rotation2d(penetration.getXDir(), penetration.getYDir()));
                Translation2d resPos = robotBumperOutline.getCenter().plus(offs);
                robotTest.setPose(new Pose2d(resPos, original.getRotation()));
                sepResult[1] = robotTest.getCenter();
                penetration.draw();

                dx = offs.getX();
                dy = offs.getY();
            } else if (SeparatingAxis.solve(robotBumperOutline, reef2, penetration)) {
                sepResult[0] = robotBumperOutline.getCenter();
                Translation2d offs = new Translation2d(penetration.getDepth(),
                    new Rotation2d(penetration.getXDir(), penetration.getYDir()));
                Translation2d resPos = robotBumperOutline.getCenter().plus(offs);
                robotTest.setPose(new Pose2d(resPos, original.getRotation()));
                sepResult[1] = robotTest.getCenter();
                penetration.draw();

                dx = offs.getX();
                dy = offs.getY();
            } else if (SeparatingAxis.solve(robotBumperOutline, centerPost, penetration)) {
                sepResult[0] = robotBumperOutline.getCenter();
                Translation2d offs = new Translation2d(penetration.getDepth(),
                    new Rotation2d(penetration.getXDir(), penetration.getYDir()));
                Translation2d resPos = robotBumperOutline.getCenter().plus(offs);
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

        if (Math.abs(dx) > 0.01 || Math.abs(dy) > 0.01) {
            Pose2d newPose = new Pose2d(x + dx, y + dy, original.getRotation());
            reset.accept(newPose);
            return newPose;
        }

        return original;
    }

    private static final Hexagon reef1 = new Hexagon("BlueReef", FieldConstants.Reef.center,
        FieldConstants.Reef.circumscribedRadius.in(Meters), Rotation2d.fromDegrees(30));
    private static final Hexagon reef2 = new Hexagon("RedReef",
        new Translation2d(FieldConstants.fieldLength.in(Meters) - FieldConstants.Reef.center.getX(),
            FieldConstants.Reef.center.getY()),
        FieldConstants.Reef.circumscribedRadius.in(Meters), Rotation2d.fromDegrees(30));
    private static final Rectangle centerPost = new Rectangle("CenterPost",
        new Pose2d(new Translation2d(FieldConstants.fieldLength.in(Meters) / 2,
            FieldConstants.fieldWidth.in(Meters) / 2), Rotation2d.kZero),
        0.4, 0.4);
    public static final Rectangle robotBumperOutline = new Rectangle("Robot", new Pose2d(),
        Constants.Swerve.bumperFront.in(Meters) * 2, Constants.Swerve.bumperRight.in(Meters) * 2);
    private static final Rectangle robotTest = new Rectangle("RobotTest", new Pose2d(),
        Constants.Swerve.bumperFront.in(Meters) * 2, Constants.Swerve.bumperRight.in(Meters) * 2);

}
