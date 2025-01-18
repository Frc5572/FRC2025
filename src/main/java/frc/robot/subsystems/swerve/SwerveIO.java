package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.swerve.SwerveModule;
import frc.lib.util.swerve.SwerveModuleIO;

/** IO Class for Swerve */
public interface SwerveIO {

    /** Inputs Class for Swerve */

    @AutoLog
    public static class SwerveInputs {
        public double yaw;
        public double roll;
        public double pitch;
    }

    public void updateInputs(SwerveInputs inputs);

    public SwerveModule[] createModules();

    public void setPose(Pose2d pose);

    public static class Empty implements SwerveIO {

        @Override
        public void updateInputs(SwerveInputs inputs) {
            // Intentionally do nothing
        }

        @Override
        public SwerveModule[] createModules() {
            return new SwerveModule[] {
                new SwerveModule(0, Rotation2d.kZero, new SwerveModuleIO.Empty()),
                new SwerveModule(1, Rotation2d.kZero, new SwerveModuleIO.Empty()),
                new SwerveModule(2, Rotation2d.kZero, new SwerveModuleIO.Empty()),
                new SwerveModule(3, Rotation2d.kZero, new SwerveModuleIO.Empty())};
        }

        @Override
        public void setPose(Pose2d pose) {
            // Intentionally do nothing
        }

    }

}
