package frc.robot.subsystems.swerve;

import com.studica.frc.AHRS;
import frc.robot.Constants;

/** IO class for NavX */
public class GyroNavX implements GyroIO {
    private AHRS gyro = new AHRS(Constants.Swerve.navXID);

    @Override
    public void updateInputs(GyroInputs inputs) {
        inputs.pitch = gyro.getPitch();
        inputs.roll = gyro.getRoll();
        inputs.yaw = gyro.getYaw();
    }
}
