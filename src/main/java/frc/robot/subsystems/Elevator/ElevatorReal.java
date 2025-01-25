package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
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
    private StatusSignal<Angle> elevatorPosition = rightElevatorMotor.getPosition();

    /** Real Elevator Initializer */
    public ElevatorReal() {
        configMotors();
    }

    public void periodic() {}

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
        rightElevatorConf.Slot0.kP = Constants.Elevator.KP;
        rightElevatorConf.Slot0.kI = Constants.Elevator.KI;
        rightElevatorConf.Slot0.kD = Constants.Elevator.KD;
        rightElevatorConf.Slot0.kS = Constants.Elevator.KS;
        rightElevatorConf.Slot0.kV = Constants.Elevator.KV;
        rightElevatorConf.Slot0.kA = Constants.Elevator.KA;
        rightElevatorConf.Slot0.kG = Constants.Elevator.KG;

        // left
        leftElevatorConf.Slot0.kP = Constants.Elevator.KP;
        leftElevatorConf.Slot0.kI = Constants.Elevator.KI;
        leftElevatorConf.Slot0.kD = Constants.Elevator.KD;
        leftElevatorConf.Slot0.kS = Constants.Elevator.KS;
        leftElevatorConf.Slot0.kV = Constants.Elevator.KV;
        leftElevatorConf.Slot0.kA = Constants.Elevator.KA;
        leftElevatorConf.Slot0.kG = Constants.Elevator.KG;

        rightElevatorMotor.getConfigurator().apply(rightElevatorConf);
        leftElevatorMotor.getConfigurator().apply(leftElevatorConf);
    }

    public void setVoltage(double volts) {
        rightElevatorMotor.setVoltage(volts);
    }

    public void setPositon(double position) {
        rightElevatorMotor.setControl(positionVoltage.withPosition(position));
    }

    /** Updates Inputs to IO */
    public void updateInputs(ElevatorInputs inputs) {
        BaseStatusSignal.refreshAll(elevatorPosition);
        inputs.limitSwitch = limitSwitch.get();
        inputs.position = elevatorPosition.getValue();
    }


}
