package frc.robot;

import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

/** Primary Drivetrain State Estimator */
public class RobotState {

    private SwerveDrivePoseEstimator swerveOdometry = null;
    private final AprilTagFieldLayout fieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    public void init(SwerveModulePosition[] positions, Rotation2d gyroYaw) {
        swerveOdometry = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, gyroYaw,
            positions, new Pose2d());
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
    public void addVisionObservation(PhotonPipelineResult result, Transform3d robotToCamera) {
        if (result.multitagResult.isPresent()) {
            Transform3d best = result.multitagResult.get().estimatedPose.best;
            Pose3d cameraPose = new Pose3d().plus(best).relativeTo(fieldLayout.getOrigin());
            Pose3d robotPose = cameraPose.plus(robotToCamera.inverse());
            Pose2d robotPose2d = robotPose.toPose2d();
            swerveOdometry.addVisionMeasurement(robotPose2d, result.getTimestampSeconds());
        }
    }

    /**
     * Add information from swerve drive.
     */
    public void addSwerveObservation(SwerveModulePosition[] positions, Rotation2d gyroYaw) {
        swerveOdometry.update(gyroYaw, positions);
    }

}
