package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Rotation;
import com.reduxrobotics.sensors.canandgyro.Canandgyro;

/** IO layer for CanandGyro */
public class GyroCanandGyro implements GyroIO {
    private Canandgyro gyro = new Canandgyro(1);

    @Override
    public void updateInputs(GyroInputs inputs) {
        inputs.yaw = Rotation.of(gyro.getYaw());
    }
}
