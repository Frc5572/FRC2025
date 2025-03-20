package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {

    @AutoLog
    public class GyroInputs {
        double yaw;
        double pitch;
        double roll;
    }

    public void updateInputs(GyroInputs inputs);

    public static class Empty implements GyroIO {
        @Override
        public void updateInputs(GyroInputs inputs) {};
    }
}
