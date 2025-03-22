package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

/** Gyro IO layer */
public interface GyroIO {
    /** Gyro input logger */
    @AutoLog
    public class GyroInputs {
        double yaw;
        double pitch;
        double roll;
    }

    public void updateInputs(GyroInputs inputs);

    /** Empty Swerve implementation (for replay) */
    public static class Empty implements GyroIO {
        @Override
        public void updateInputs(GyroInputs inputs) {}
    }
}
