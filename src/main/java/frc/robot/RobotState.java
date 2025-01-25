package frc.robot;

import java.util.stream.Stream;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.lib.util.viz.Viz2025;

/** Primary Drivetrain State Estimator */
public class RobotState {

    private final Viz2025 vis;

    public RobotState(Viz2025 vis) {
        this.vis = vis;
    }

    private SwerveDrivePoseEstimator swerveOdometry = null;


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
    public void addVisionObservation(PhotonPipelineResult result, Transform3d robotToCamera,
        int whichCamera) {
        if (result.multitagResult.isPresent()) {
            Transform3d best = result.multitagResult.get().estimatedPose.best;
            Pose3d cameraPose =
                new Pose3d().plus(best).relativeTo(Constants.Vision.fieldLayout.getOrigin());
            Pose3d robotPose = cameraPose.plus(robotToCamera.inverse());
            Pose2d robotPose2d = robotPose.toPose2d();
            Logger.recordOutput("State/GlobalVisionEstimate", robotPose);
            swerveOdometry.addVisionMeasurement(robotPose2d, result.getTimestampSeconds(),
                VecBuilder.fill(0.02, 0.02, 0.02));
        }
    }

    /**
     * Add information from swerve drive.
     */
    public void addSwerveObservation(SwerveModulePosition[] positions, Rotation2d gyroYaw) {
        swerveOdometry.update(gyroYaw, positions);
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

}
