package frc.robot.subsystems.swerve;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import com.studica.frc.AHRS;
import frc.robot.Constants;

public class GyroReal implements GyroIO {

    private Canandgyro gyro = new Canandgyro(1);
    private AHRS navX = new AHRS(Constants.Swerve.navXID);

    public GyroReal() {}

    @Override
    public void updateInputs(GyroInputs inputs) {
        inputs.yawCanAndGyro = gyro.getYaw();
        inputs.pitchCanAndGyro = gyro.getPitch();
        inputs.rollCanAndGyro = gyro.getRoll();
        inputs.yawNavX = navX.getYaw();
        inputs.pitchNavX = navX.getPitch();
        inputs.rollNavX = navX.getRoll();
    }
}
