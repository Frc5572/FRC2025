package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {

    @AutoLog
    public static class GyroInputs {
        public double yawCanAndGyro;
        public double rollCanAndGyro;
        public double pitchCanAndGyro;
        public double yawNavX;
        public double rollNavX;
        public double pitchNavX;
    }

    public void updateInputs(GyroInputs inputs);

    public static class Empty implements GyroIO {

        @Override
        public void updateInputs(GyroInputs inputs) {}
    }
}
