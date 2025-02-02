package frc.lib.util.swerve;

import static edu.wpi.first.units.Units.Rotations;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.lib.mut.MutableRotation2d;
import frc.lib.mut.MutableSwerveModulePosition;
import frc.lib.mut.MutableSwerveModuleState;
import frc.robot.Constants;

/**
 * Swerve Module Subsystem
 */
public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private SwerveModuleIO io;
    private SwerveModuleInputsAutoLogged inputs = new SwerveModuleInputsAutoLogged();

    /**
     * Swerve Module
     *
     * @param moduleNumber Module Number
     * @param angleOffset Angle Offset of the CANCoder to align the wheels
     */
    public SwerveModule(int moduleNumber, Rotation2d angleOffset, SwerveModuleIO io) {
        this.io = io;

        this.moduleNumber = moduleNumber;

        this.angleOffset = angleOffset;

        // lastAngle = getState().angle.getDegrees();
        io.updateInputs(inputs);
        Logger.processInputs("SwerveModule" + moduleNumber, inputs);
    }

    /**
     * Update inputs for a Swerve Module.
     */
    public void periodic() {
        // Robot.profiler.push("updateInputs");
        io.updateInputs(inputs);
        // Robot.profiler.swap("processInputs");
        Logger.processInputs("SwerveModule" + moduleNumber, inputs);
        // Robot.profiler.pop();
    }

    /**
     * Set the desired state of the Swerve Module
     *
     * @param desiredState The desired {@link SwerveModuleState} for the module
     * @param isOpenLoop Whether the state should be open or closed loop controlled
     */
    public void setDesiredState(MutableSwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState.optimize(getState().angle);
        io.setAngleMotor(desiredState.angle.getRotations());
        setSpeed(desiredState, isOpenLoop);
    }

    /**
     * Set the velocity or power of the drive motor
     *
     * @param desiredState The desired {@link SwerveModuleState} of the module
     * @param isOpenLoop Whether the state should be open or closed loop controlled
     */
    private void setSpeed(MutableSwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double power = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            io.setDriveMotorPower(power);
        } else {
            io.setDriveMotor(desiredState.speedMetersPerSecond);
        }
    }

    private final MutableRotation2d canCoderCurrent = new MutableRotation2d();

    /**
     * Get the rotation of the CANCoder
     *
     * @return The rotation of the CANCoder in {@link MutableRotation2d}
     */
    public MutableRotation2d getCANcoder() {
        canCoderCurrent.setAngle(inputs.absolutePositionAngleEncoder);
        return canCoderCurrent;
    }

    private final MutableRotation2d angle = new MutableRotation2d();

    private MutableRotation2d getAngle() {
        angle.setRotations(inputs.angleMotorSelectedPosition.in(Rotations));
        return angle;
    }

    private final MutableSwerveModuleState state = new MutableSwerveModuleState(0.0, angle);
    private final MutableSwerveModulePosition position =
        new MutableSwerveModulePosition(0.0, angle);

    /**
     * Get the current Swerve Module State
     *
     * @return The current {@link MutableSwerveModuleState}
     */
    public MutableSwerveModuleState getState() {
        getAngle();
        state.speedMetersPerSecond = Conversions.rotationPerSecondToMetersPerSecond(
            inputs.driveMotorSelectedSensorVelocity, Constants.Swerve.wheelCircumference);
        return state;
    }

    /**
     * Get the current Swerve Module Position
     *
     * @return The current {@link SwerveModulePosition}
     */
    public MutableSwerveModulePosition getPosition() {
        getAngle();
        position.distanceMeters = Conversions.rotationsToMeters(inputs.driveMotorSelectedPosition,
            Constants.Swerve.wheelCircumference);
        return position;
    }
}
