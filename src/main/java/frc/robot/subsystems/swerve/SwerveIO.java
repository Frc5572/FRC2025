package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * DrivetrainIO interface
 */
public interface SwerveIO {
    /**
     * Drivetrain IO
     */
    @AutoLog
    public static class SwerveInputs {
        public Rotation2d gyroYaw = new Rotation2d();
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(SwerveInputs inputs) {}

    /** Run the motor at the specified voltage. */
    public default void setDriveVoltage(double lvolts, double rvolts) {}


}
