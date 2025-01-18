package frc.lib.util.swerve;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import frc.lib.math.Conversions;
import frc.robot.Constants;

public class SwerveModuleSim implements SwerveModuleIO {

    private final SwerveModuleSimulation moduleSimulation;
    private final SimulatedMotorController.GenericMotorController driveMotor;
    private final SimulatedMotorController.GenericMotorController turnMotor;

    private final int id;

    private boolean driveClosedLoop = false;
    private boolean turnClosedLoop = false;
    private final PIDController driveController;
    private final PIDController turnController;
    private double driveFFVolts = 0.0;
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    public SwerveModuleSim(int id, SwerveModuleSimulation modSim) {
        this.id = id;
        this.moduleSimulation = modSim;
        this.driveMotor = moduleSimulation.useGenericMotorControllerForDrive()
            .withCurrentLimit(Constants.Swerve.ModuleConstants.slipCurrent);
        this.turnMotor =
            moduleSimulation.useGenericControllerForSteer().withCurrentLimit(Units.Amps.of(20));

        this.driveController = new PIDController(0.5, 0.0, 0.0);
        this.turnController = new PIDController(8.0, 0.0, 0.0);

        // Enable wrapping for turn PID
        turnController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(SwerveModuleInputs inputs) {
        if (driveClosedLoop) {
            driveAppliedVolts = driveFFVolts + driveController
                .calculate(moduleSimulation.getDriveWheelFinalSpeed().in(Units.RadiansPerSecond));
        } else {
            driveController.reset();
        }
        if (turnClosedLoop) {
            turnAppliedVolts =
                turnController.calculate(moduleSimulation.getSteerAbsoluteFacing().getRadians());
        } else {
            turnController.reset();
        }

        Logger.recordOutput("driveVoltage" + id, driveAppliedVolts);
        Logger.recordOutput("angleVoltage" + id, turnAppliedVolts);

        driveMotor.requestVoltage(Units.Volts.of(driveAppliedVolts));
        turnMotor.requestVoltage(Units.Volts.of(turnAppliedVolts));

        inputs.driveMotorSelectedPosition = moduleSimulation.getDriveWheelFinalPosition();
        inputs.driveMotorSelectedSensorVelocity = moduleSimulation.getDriveWheelFinalSpeed();
        inputs.absolutePositionAngleEncoder =
            moduleSimulation.getSteerAbsoluteFacing().getMeasure();
        inputs.angleMotorSelectedPosition = moduleSimulation.getSteerAbsoluteFacing().getMeasure();
    }

    @Override
    public void setDriveMotor(double mps) {
        driveClosedLoop = true;
        double radps =
            Conversions.metersPerSecondToRotationPerSecond(mps, Constants.Swerve.wheelCircumference)
                * Math.PI * 2.0;
        driveController.setSetpoint(radps);
    }

    @Override
    public void setDriveMotorPower(double power) {
        driveClosedLoop = false;
        driveAppliedVolts = power;
    }

    @Override
    public void setAngleMotor(double angle) {
        turnClosedLoop = true;
        turnController.setSetpoint(edu.wpi.first.math.util.Units.rotationsToRadians(angle));
    }

    @Override
    public void setPositionAngleMotor(double absolutePosition) {

    }

}
