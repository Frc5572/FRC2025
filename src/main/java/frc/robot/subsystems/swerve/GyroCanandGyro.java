package frc.robot.subsystems.swerve;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.util.swerve.SwerveModule;

public class GyroCanandGyro implements SwerveIO {
    private Canandgyro gyro = new Canandgyro(1);

    @Override
    public void updateInputs(SwerveInputs inputs) {
        inputs.pitch = gyro.getPitch();
        inputs.roll = gyro.getRoll();
        inputs.yaw = gyro.getYaw();
    }

    @Override
    public SwerveModule[] createModules() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'createModules'");
    }

    @Override
    public void setPose(Pose2d pose) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setPose'");
    }
}
