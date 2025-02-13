package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

/**
 * Real Class for Elevator
 */
public class elreal implements elio {
    private final TalonFX rightElevatorMotor = new TalonFX(Constants.Elevator.RIGHT_ID);
    private final TalonFX leftElevatorMotor = new TalonFX(Constants.Elevator.LEFT_ID);
    private final DigitalInput limitSwitch = new DigitalInput(Constants.Elevator.LIMIT_ID);
    private final TalonFXConfiguration rightElevatorConf = new TalonFXConfiguration();
    private final TalonFXConfiguration leftElevatorConf = new TalonFXConfiguration();
    private final PositionVoltage positionVoltage = new PositionVoltage(0.0).withSlot(0);
    private StatusSignal<Angle> elevatorPosition = rightElevatorMotor.getPosition();
    private StatusSignal<Voltage> elevatorVoltage = rightElevatorMotor.getMotorVoltage();
    private StatusSignal<AngularVelocity> elevatorVelocity = rightElevatorMotor.getVelocity();

    /** Real Elevator Initializer */
    public elreal() {
        configMotors();
    }

    public void periodic() {}

    private void configMotors() {
        // left conf

        leftElevatorMotor.setControl(new Follower(rightElevatorMotor.getDeviceID(), true));
        leftElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        leftElevatorConf.Feedback.SensorToMechanismRatio =
            Constants.Elevator.SensorToMechanismRatio;

        // right conf
        rightElevatorConf.MotorOutput.NeutralMode = Constants.Elevator.BREAK;
        leftElevatorConf.MotorOutput.NeutralMode = Constants.Elevator.BREAK;

        rightElevatorConf.Feedback.SensorToMechanismRatio =
            Constants.Elevator.SensorToMechanismRatio;


        // PID and feedforward

        // right
        rightElevatorConf.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
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
        BaseStatusSignal.refreshAll(elevatorPosition, elevatorVelocity, elevatorVoltage);
        inputs.limitSwitch = !limitSwitch.get();
        inputs.position = Meters.of(elevatorPosition.getValue().in(Rotations));
        inputs.velocity = elevatorVelocity.getValue();
        inputs.outputVoltage = elevatorVoltage.getValue();
    }

    public void resetHome() {
        rightElevatorMotor.setPosition(0);
    }


}
