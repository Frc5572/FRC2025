package frc.robot.subsystems.swerve;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;

/** IO layer for CanandGyro */
public class GyroCanandGyro implements GyroIO {
    private Canandgyro gyro = new Canandgyro(1);

    @Override
    public void updateInputs(GyroInputs inputs) {
        inputs.pitch = gyro.getPitch();
        inputs.roll = gyro.getRoll();
        inputs.yaw = gyro.getYaw();
    }
}
