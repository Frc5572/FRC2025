package frc.robot.subsystems.swerve;


import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.swerve.SwerveModule;
import frc.lib.util.swerve.SwerveModuleReal;
import frc.robot.Constants;

/** Real Class for Swerve */
public class SwerveReal implements SwerveIO {

    private Canandgyro gyro = new Canandgyro(1);
    private AHRS navX = new AHRS(Constants.Swerve.navXID);

    /** Real Swerve Initializer */
    public SwerveReal() {
        navX.setAngleAdjustment(180);
    }

    @Override
    public void updateInputs(SwerveInputs inputs) {
        inputs.yawCanAndGyro = gyro.getYaw();
        inputs.pitchCanAndGyro = gyro.getPitch();
        inputs.rollCanAndGyro = gyro.getRoll();
        inputs.yawNavX = navX.getYaw();
        inputs.pitchNavX = navX.getPitch();
        inputs.rollNavX = navX.getRoll();

    }

    public SwerveModule createSwerveModule(int moduleNumber, int driveMotorID, int angleMotorID,
        int cancoderID, Rotation2d angleOffset) {
        return new SwerveModule(moduleNumber, angleOffset,
            new SwerveModuleReal(driveMotorID, angleMotorID, cancoderID, angleOffset));
    }

    @Override
    public SwerveModule[] createModules() {
        return new SwerveModule[] {
            createSwerveModule(0, Constants.Swerve.Mod0.driveMotorID,
                Constants.Swerve.Mod0.angleMotorID, Constants.Swerve.Mod0.canCoderID,
                Constants.Swerve.Mod0.angleOffset),
            createSwerveModule(1, Constants.Swerve.Mod1.driveMotorID,
                Constants.Swerve.Mod1.angleMotorID, Constants.Swerve.Mod1.canCoderID,
                Constants.Swerve.Mod1.angleOffset),
            createSwerveModule(2, Constants.Swerve.Mod2.driveMotorID,
                Constants.Swerve.Mod2.angleMotorID, Constants.Swerve.Mod2.canCoderID,
                Constants.Swerve.Mod2.angleOffset),
            createSwerveModule(3, Constants.Swerve.Mod3.driveMotorID,
                Constants.Swerve.Mod3.angleMotorID, Constants.Swerve.Mod3.canCoderID,
                Constants.Swerve.Mod3.angleOffset)};
    }

    @Override
    public void setPose(Pose2d pose) {
        // Intentionally do nothing
    }

}
