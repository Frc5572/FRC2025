package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * DrivetrainReal
 */
public class SwerveReal implements SwerveIO {


    /**
     * Drivetrain Real
     */
    public SwerveReal() {}

    @Override
    public void updateInputs(SwerveInputs inputs) {
        inputs.gyroYaw = Rotation2d.fromDegrees(0);
    }

    /**
     * Drive Voltage
     */
    public void setDriveVoltage(double lvolts, double rvolts) {}

}
