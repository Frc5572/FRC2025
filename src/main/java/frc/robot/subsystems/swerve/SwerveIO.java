package frc.robot.subsystems.swerve;

import java.util.Optional;
import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.util.swerve.SwerveModule;

/** IO Class for Swerve */
public interface SwerveIO {

    /** Inputs Class for Swerve */

    @AutoLog
    public static class SwerveInputs {
        public double yaw;
        public double roll;
        public double pitch;
    }

    public default void updateInputs(SwerveInputs inputs) {}

    public default SwerveModule[] createModules() {
        return new SwerveModule[] {};
    }

    public default Optional<Pose2d> getInitialPose() {
        return Optional.empty();
    }

    public default void setPose(Pose2d pose) {}

}
