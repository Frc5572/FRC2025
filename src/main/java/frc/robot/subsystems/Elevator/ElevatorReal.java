package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;


public class ElevatorReal implements ElevatorIO {
    private final TalonFX rightElevatorMotor = new TalonFX(Constants.Elevator.RIGHT_ID);
    private final TalonFX leftElevatorMotor = new TalonFX(Constants.Elevator.LEFT_ID);
    private final DigitalInput limitSwitch = new DigitalInput(Constants.Elevator.LIMIT_ID);
    private final TalonFXConfiguration rightElevatorConf = new TalonFXConfiguration();
    private final TalonFXConfiguration leftElevatorConf = new TalonFXConfiguration();
    private final PositionVoltage positionVoltage = new PositionVoltage(0.0);

    public ElevatorReal() {}

    private void configMotors() {
        // left conf
        leftElevatorConf.MotorOutput.NeutralMode = null;

        leftElevatorMotor.setControl(new Follower(rightElevatorMotor.getDeviceID(), true));
        leftElevatorMotor.setNeutralMode(NeutralModeValue.Brake);

        // right conf
        rightElevatorConf.MotorOutput.NeutralMode = Constants.Elevator.RIGHT_BREAK;

        // PID and feedforward
        rightElevatorConf.Slot0.kP = 0.0;
        rightElevatorConf.Slot0.kI = 0.0;
        rightElevatorConf.Slot0.kD = 0.0;
        rightElevatorConf.Slot0.kS = 0.0;
        rightElevatorConf.Slot0.kV = 0.0;
        rightElevatorConf.Slot0.kA = 0.0;
        rightElevatorConf.Slot0.kG = 0.0;

        leftElevatorConf.Slot0.kP = 0.0;
        leftElevatorConf.Slot0.kI = 0.0;
        leftElevatorConf.Slot0.kD = 0.0;
        leftElevatorConf.Slot0.kS = 0.0;
        leftElevatorConf.Slot0.kV = 0.0;
        leftElevatorConf.Slot0.kA = 0.0;
        leftElevatorConf.Slot0.kG = 0.0;

        rightElevatorMotor.getConfigurator().apply(rightElevatorConf);
        leftElevatorMotor.getConfigurator().apply(leftElevatorConf);
    }

    public void setVoltage(double volts) {
        rightElevatorMotor.setVoltage(volts);
    }

    public void setPositon(double position) {
        rightElevatorMotor.setControl(positionVoltage.withPosition(position));
    }

    public void updateInputs(ElevatorInputs inputs) {
        inputs.limitSwitch = limitSwitch.get();
    }


}
