package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Degrees;
import com.studica.frc.AHRS;
import frc.robot.Constants;

/** IO class for NavX */
public class GyroNavX implements GyroIO {
    private AHRS gyro = new AHRS(Constants.Swerve.navXID);

    @Override
    public void updateInputs(GyroInputs inputs) {
        inputs.yaw = Degrees.of(-gyro.getYaw());
    }
}
