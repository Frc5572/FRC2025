
package frc.robot.subsystems.swerve.mod;

import static edu.wpi.first.units.Units.Meters;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.lib.util.LoggedTunableNumber;
import frc.robot.Constants;

/** Swerve Module Subsystem */
public final class SwerveModule {

    private static final LoggedTunableNumber drivekP =
        new LoggedTunableNumber("Drive/Module/DrivekP", Constants.Swerve.ModuleConstants.drivekP);
    private static final LoggedTunableNumber drivekD =
        new LoggedTunableNumber("Drive/Module/DrivekD", Constants.Swerve.ModuleConstants.drivekD);
    private static final LoggedTunableNumber drivekS =
        new LoggedTunableNumber("Drive/Module/DrivekS", Constants.Swerve.ModuleConstants.ffkS);
    private static final LoggedTunableNumber drivekV =
        new LoggedTunableNumber("Drive/Module/DrivekV", Constants.Swerve.ModuleConstants.ffkV);
    private static final LoggedTunableNumber drivekT =
        new LoggedTunableNumber("Drive/Module/DrivekT", Constants.Swerve.ModuleConstants.ffkT);
    private static final LoggedTunableNumber anglekP =
        new LoggedTunableNumber("Drive/Module/AnglekP", Constants.Swerve.ModuleConstants.anglekP);
    private static final LoggedTunableNumber anglekD =
        new LoggedTunableNumber("Drive/Module/AnglekD", Constants.Swerve.ModuleConstants.anglekD);

    private final SwerveModuleAngleIO angleIO;
    private final SwerveModuleDriveIO driveIO;

    private final String angleKey;
    private final String driveKey;

    /** Inputs from the angle motor */
    public final SwerveModuleAngleInputsAutoLogged angleInputs =
        new SwerveModuleAngleInputsAutoLogged();

    /** Inputs from the drive motor */
    public final SwerveModuleDriveInputsAutoLogged driveInputs =
        new SwerveModuleDriveInputsAutoLogged();

    private SimpleMotorFeedforward ffModel;

    private final Alert driveDisconnectedAlert;
    private final Alert angleDisconnectedAlert;
    private final Alert angleEncoderDisconnectedAlert;
    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

    /** Swerve Module */
    public SwerveModule(ModuleConfig config, SwerveModuleAngleIO angleIO,
        SwerveModuleDriveIO driveIO) {
        this.angleIO = angleIO;
        this.driveIO = driveIO;
        int index = config.moduleNumber;
        this.angleKey = "Swerve/Mod[" + index + "]/angle";
        this.driveKey = "Swerve/Mod[" + index + "]/drive";

        ffModel = new SimpleMotorFeedforward(drivekS.get(), drivekV.get());

        driveDisconnectedAlert =
            new Alert("Disconnected drive motor on module " + index + ".", AlertType.kError);
        angleDisconnectedAlert =
            new Alert("Disconnected angle motor on module " + index + ".", AlertType.kError);
        angleEncoderDisconnectedAlert =
            new Alert("Disconnected angle encoder on module " + index + ".", AlertType.kError);
    }

    /**
     * Update inputs for a Swerve Module. Run while the odometryLock is held.
     */
    public void updateInputs() {
        angleIO.updateAngleInputs(angleInputs);
        driveIO.updateDriveInputs(driveInputs);
        Logger.processInputs(angleKey, angleInputs);
        Logger.processInputs(driveKey, driveInputs);
    }

    /**
     * Periodic function. Run while the odometryLock is not held.
     */
    public void periodic() {
        Logger.recordOutput(this.driveKey, driveInputs.odometryDrivePositionsMeters.length);
        Logger.recordOutput(this.driveKey, driveInputs.odometryTimestamps.length);
        Logger.recordOutput(this.angleKey, angleInputs.odometryTurnPositions.length);
        // Update tunable numbers
        if (drivekS.hasChanged(hashCode()) || drivekV.hasChanged(hashCode())) {
            ffModel = new SimpleMotorFeedforward(drivekS.get(), drivekV.get());
        }
        if (drivekP.hasChanged(hashCode()) || drivekD.hasChanged(hashCode())) {
            driveIO.setDrivePID(drivekP.get(), 0, drivekD.get());
        }
        if (anglekP.hasChanged(hashCode()) || anglekD.hasChanged(hashCode())) {
            driveIO.setDrivePID(anglekP.get(), 0, anglekD.get());
        }
        int sampleCount = driveInputs.odometryTimestamps.length;
        // Calculate positions for odometry
        odometryPositions = new SwerveModulePosition[sampleCount];
        for (int i = 0; i < sampleCount; i++) {
            double positionMeters = driveInputs.odometryDrivePositionsMeters[i]
                * Constants.Swerve.ModuleConstants.wheelRadius.in(Meters);
            Rotation2d angle = angleInputs.odometryTurnPositions[i];
            odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        }

        // Update alerts
        driveDisconnectedAlert.set(!driveInputs.motorConnected);
        angleDisconnectedAlert.set(!angleInputs.motorConnected);
        angleEncoderDisconnectedAlert.set(!angleInputs.encoderConnected);
    }

    /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
    public void runSetpoint(SwerveModuleState state) {
        runSetpoint(state, 0);
    }

    /**
     * runs the module with the specified setpoint state and a setpoint wheel torque used for
     * feedforward. Mutates the state to optimize it.
     */
    public void runSetpoint(SwerveModuleState state, double wheelTorqueNm) {
        // Optimize velocity setpoint
        state.optimize(getAngle());
        state.cosineScale(angleInputs.position);

        // Apply setpoints
        double speedRadPerSec = state.speedMetersPerSecond
            / Constants.Swerve.ModuleConstants.wheelRadius.in(edu.wpi.first.units.Units.Meters);
        driveIO.runDriveVelocity(speedRadPerSec,
            ffModel.calculate(speedRadPerSec) + wheelTorqueNm * drivekT.get());
        angleIO.runAnglePosition(state.angle);
    }

    /** Runs the module with the specified output while controlling to zero degrees. */
    public void runCharacterization(double output) {
        driveIO.runDriveOpenLoop(output);
        angleIO.runAnglePosition(Rotation2d.kZero);
    }

    /** Disables all outputs to motors. */
    public void stop() {
        driveIO.runDriveOpenLoop(0.0);
        angleIO.runAngleOpenLoop(0.0);
    }

    /** Returns the current turn angle of the module. */
    public Rotation2d getAngle() {
        return angleInputs.position;
    }

    /** Returns the current drive position of the module in meters. */
    public double getPositionMeters() {
        return driveInputs.positionRads
            * Constants.Swerve.ModuleConstants.wheelRadius.in(edu.wpi.first.units.Units.Meters);
    }

    /** Returns the current drive velocity of the module in meters per second. */
    public double getVelocityMetersPerSec() {
        return driveInputs.velocityRadsPerSec
            * Constants.Swerve.ModuleConstants.wheelRadius.in(edu.wpi.first.units.Units.Meters);
    }

    /** Returns the module position (turn angle and drive position). */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    /** Returns the module state (turn angle and drive velocity). */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
    }

    /** Returns the module positions received this cycle. */
    public SwerveModulePosition[] getOdometryPositions() {
        return odometryPositions;
    }

    /** Returns the module position in radians. */
    public double getWheelRadiusCharacterizationPosition() {
        return driveInputs.positionRads;
    }

    /** Returns the module velocity in rotations/sec (Phoenix native units). */
    public double getFFCharacterizationVelocity() {
        return Units.radiansToRotations(driveInputs.velocityRadsPerSec);
    }

}
