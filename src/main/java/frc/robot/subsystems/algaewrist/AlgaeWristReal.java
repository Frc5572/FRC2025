package frc.robot.subsystems.algaewrist;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class AlgaeWristReal implements AlgaeWristIO {

    private final TalonFX wristMotor =
        new TalonFX(Constants.Motors.AlgaeMotors.ALGAE_WRIST_MOTOR_ID);
    private final TalonFXConfiguration wristConf = new TalonFXConfiguration();
    private StatusSignal<Angle> wristPosition = wristMotor.getPosition();
    private StatusSignal<Voltage> wristVoltage = wristMotor.getMotorVoltage();
    private StatusSignal<Current> motorCurrent = wristMotor.getStatorCurrent();

    private final CANcoder angleEncoder = new CANcoder(Constants.Algae.CANCODER_ID);
    private final CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
    private final StatusSignal<Angle> absoultePosition = angleEncoder.getAbsolutePosition();

    public AlgaeWristReal() {
        configAngleEncoder();
        configMotor();

        BaseStatusSignal.setUpdateFrequencyForAll(50, wristPosition, wristVoltage, motorCurrent,
            absoultePosition);
        ParentDevice.optimizeBusUtilizationForAll(wristMotor, angleEncoder);

        // wristMotor.setPosition(Degrees.of(90));
    }

    private void configAngleEncoder() {
        cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        cancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        cancoderConfig.MagnetSensor.MagnetOffset = 0.457031;

        angleEncoder.getConfigurator().apply(cancoderConfig);
    }

    private void configMotor() {
        wristConf.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        wristConf.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Gear ratio
        wristConf.Feedback.FeedbackRemoteSensorID = angleEncoder.getDeviceID();
        wristConf.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        wristConf.Feedback.SensorToMechanismRatio = 1.0;
        wristConf.Feedback.RotorToSensorRatio = 180.0;
        wristConf.ClosedLoopGeneral.ContinuousWrap = true;

        wristConf.Slot0.kP = 80.0;
        wristConf.Slot0.kI = 0.0;
        wristConf.Slot0.kD = 0.0;

        wristConf.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        wristConf.Slot0.kG = 0.0;
        wristConf.Slot0.kS = 0.3;

        wristMotor.getConfigurator().apply(wristConf);
    }

    @Override
    public void updateInputs(AlgaeWristInputs inputs) {
        BaseStatusSignal.refreshAll(wristPosition, wristVoltage, motorCurrent, absoultePosition);
        inputs.wristAngle = wristPosition.getValue();
        inputs.wristAngleAbsolute = absoultePosition.getValue();
        inputs.voltage = wristVoltage.getValue();
        inputs.current = motorCurrent.getValue();
    }

    private final PositionVoltage mmVoltage = new PositionVoltage(0);

    @Override
    public void setWristSetpoint(Angle angle) {
        wristMotor.setControl(mmVoltage.withPosition(angle));
    }

    private final VoltageOut voltage = new VoltageOut(0.0);

    @Override
    public void setWristVoltage(double volts) {
        wristMotor.setControl(voltage.withOutput(volts));
    }

    @Override
    public void setBrakeMode(boolean brake) {
        wristConf.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        wristMotor.getConfigurator().apply(wristConf);
    }

}
