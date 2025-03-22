package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

/**
 * Real Class for Elevator
 */
public class ElevatorReal implements ElevatorIO {
    private final TalonFX rightElevatorMotor = new TalonFX(Constants.Elevator.RIGHT_ID);
    private final TalonFX leftElevatorMotor = new TalonFX(Constants.Elevator.LEFT_ID);
    private final DigitalInput limitSwitch = new DigitalInput(Constants.Elevator.LIMIT_ID);
    private final TalonFXConfiguration elevatorConf = new TalonFXConfiguration();
    private StatusSignal<Angle> elevatorPosition = rightElevatorMotor.getPosition();
    private StatusSignal<Voltage> elevatorVoltage = rightElevatorMotor.getMotorVoltage();
    private StatusSignal<AngularVelocity> elevatorVelocity = rightElevatorMotor.getVelocity();
    private StatusSignal<Current> motorCurrent = rightElevatorMotor.getStatorCurrent();
    private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
    private final DynamicMotionMagicVoltage m_requestFast = new DynamicMotionMagicVoltage(0,
        Constants.Elevator.FastVelocity, Constants.Elevator.Acceleration, Constants.Elevator.Jerk);

    /** Real Elevator Initializer */
    public ElevatorReal() {
        configMotors();
    }

    public void periodic() {}

    private void configMotors() {
        // left conf

        leftElevatorMotor.setControl(new Follower(rightElevatorMotor.getDeviceID(), true));

        // right conf
        elevatorConf.MotorOutput.NeutralMode = Constants.Elevator.BREAK;

        elevatorConf.Feedback.SensorToMechanismRatio = Constants.Elevator.SensorToMechanismRatio;


        // PID and feedforward

        // right
        elevatorConf.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        elevatorConf.Slot0.kP = Constants.Elevator.KP;
        elevatorConf.Slot0.kI = Constants.Elevator.KI;
        elevatorConf.Slot0.kD = Constants.Elevator.KD;
        elevatorConf.Slot0.kS = Constants.Elevator.KS;
        elevatorConf.Slot0.kV = Constants.Elevator.KV;
        elevatorConf.Slot0.kA = Constants.Elevator.KA;
        elevatorConf.Slot0.kG = Constants.Elevator.KG;
        elevatorConf.MotionMagic.MotionMagicCruiseVelocity = Constants.Elevator.CVeleocity;
        elevatorConf.MotionMagic.MotionMagicAcceleration = Constants.Elevator.Acceleration;
        elevatorConf.MotionMagic.MotionMagicJerk = Constants.Elevator.Jerk;


        rightElevatorMotor.getConfigurator().apply(elevatorConf);
        leftElevatorMotor.getConfigurator().apply(elevatorConf);
    }

    public void setVoltage(double volts) {
        rightElevatorMotor.setVoltage(volts);
    }

    public void setPower(double power) {
        rightElevatorMotor.set(power);;
    }

    public void setPositon(double position) {
        rightElevatorMotor.setControl(m_request.withPosition(position));
    }


    /** Updates Inputs to IO */
    public void updateInputs(ElevatorInputs inputs) {
        BaseStatusSignal.refreshAll(elevatorPosition, elevatorVelocity, elevatorVoltage,
            motorCurrent);
        inputs.limitSwitch = limitSwitch.get();
        inputs.position = Meters.of(elevatorPosition.getValue().in(Rotations));
        inputs.velocity = elevatorVelocity.getValue();
        inputs.outputVoltage = elevatorVoltage.getValue();
        inputs.motorCurrent = motorCurrent.getValue();
    }

    public void resetHome() {
        rightElevatorMotor.setPosition(0);
    }

    @Override
    public void setPositonFast(double position) {
        rightElevatorMotor.setControl(m_requestFast.withPosition(position));
    }


}
