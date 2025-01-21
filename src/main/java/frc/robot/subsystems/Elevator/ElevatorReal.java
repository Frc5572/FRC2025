package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

/**
 * Real Class for Elevator
 */
public class ElevatorReal implements ElevatorIO {
    private final TalonFX rightElevatorMotor = new TalonFX(Constants.Elevator.RIGHT_ID);
    private final TalonFX leftElevatorMotor = new TalonFX(Constants.Elevator.LEFT_ID);
    private final DigitalInput limitSwitch = new DigitalInput(Constants.Elevator.LIMIT_ID);
    private final TalonFXConfiguration rightElevatorConf = new TalonFXConfiguration();
    private final TalonFXConfiguration leftElevatorConf = new TalonFXConfiguration();
    private final PositionVoltage positionVoltage = new PositionVoltage(0.0);

    /** Real Elevator Initilizer */
    public ElevatorReal() {
        configMotors();
    }

    private void configMotors() {
        // left conf
        leftElevatorConf.MotorOutput.NeutralMode = null;

        leftElevatorMotor.setControl(new Follower(rightElevatorMotor.getDeviceID(), true));
        leftElevatorMotor.setNeutralMode(NeutralModeValue.Brake);

        // right conf
        rightElevatorConf.MotorOutput.NeutralMode = Constants.Elevator.BREAK;
        leftElevatorConf.MotorOutput.NeutralMode = Constants.Elevator.BREAK;

        // PID and feedforward

        // right
        rightElevatorConf.Slot0.kP = Constants.Elevator.RIGHT_KP;
        rightElevatorConf.Slot0.kI = Constants.Elevator.RIGHT_KI;
        rightElevatorConf.Slot0.kD = Constants.Elevator.RIGHT_KD;
        rightElevatorConf.Slot0.kS = Constants.Elevator.RIGHT_KS;
        rightElevatorConf.Slot0.kV = Constants.Elevator.RIGHT_KV;
        rightElevatorConf.Slot0.kA = Constants.Elevator.RIGHT_KA;
        rightElevatorConf.Slot0.kG = Constants.Elevator.RIGHT_KG;

        // left
        leftElevatorConf.Slot0.kP = Constants.Elevator.RIGHT_KP;
        leftElevatorConf.Slot0.kI = Constants.Elevator.RIGHT_KI;
        leftElevatorConf.Slot0.kD = Constants.Elevator.RIGHT_KD;
        leftElevatorConf.Slot0.kS = Constants.Elevator.RIGHT_KS;
        leftElevatorConf.Slot0.kV = Constants.Elevator.RIGHT_KV;
        leftElevatorConf.Slot0.kA = Constants.Elevator.RIGHT_KA;
        leftElevatorConf.Slot0.kG = Constants.Elevator.RIGHT_KG;

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
