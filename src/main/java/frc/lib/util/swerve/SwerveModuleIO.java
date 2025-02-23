package frc.lib.util.swerve;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/** IO Class for SwerveModule */
public interface SwerveModuleIO {
    /** Inputs Class for SwerveModule */
    @AutoLog
    public static class SwerveModuleInputs {
        public Angle driveMotorSelectedPosition = Rotations.of(0.0);
        public AngularVelocity driveMotorSelectedSensorVelocity = RotationsPerSecond.of(0.0);
        public Angle angleMotorSelectedPosition = Rotations.of(0.0);
        public Angle absolutePositionAngleEncoder = Rotations.of(0.0);
        // public double driveMotorTemp;
        // public double angleMotorTemp;
    }

    public void updateInputs(SwerveModuleInputs inputs);

    public void setDriveMotor(double mps);

    public void setDriveMotorPower(double power);

    public void setAngleMotor(double angle);

    public void setPositionAngleMotor(double absolutePosition);

    /** Empty implementation of a Swerve Module (for replay) */
    public static class Empty implements SwerveModuleIO {

        @Override
        public void updateInputs(SwerveModuleInputs inputs) {
            // Intentionally do nothing
        }

        @Override
        public void setDriveMotor(double mps) {
            // Intentionally do nothing
        }

        @Override
        public void setDriveMotorPower(double power) {
            // Intentionally do nothing
        }

        @Override
        public void setAngleMotor(double angle) {
            // Intentionally do nothing
        }

        @Override
        public void setPositionAngleMotor(double absolutePosition) {
            // Intentionally do nothing
        }

    }

}
