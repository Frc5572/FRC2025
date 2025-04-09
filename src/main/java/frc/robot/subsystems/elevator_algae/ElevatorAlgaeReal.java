package frc.robot.subsystems.elevator_algae;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;

/**
 * Algae Real Class
 */
public class ElevatorAlgaeReal implements ElevatorAlgaeIO {
    private final SparkFlex algaeMotor =
        new SparkFlex(Constants.Motors.AlgaeMotors.ALGAE_ROLLER_MOTOR_ID, MotorType.kBrushless);// Algae
    // motor
    private final TalonFX pivotMotor = new TalonFX(0);
    private final TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
    private final SparkFlexConfig algaeMotorConfig = new SparkFlexConfig();
    private final CANcoder canCoder = new CANcoder(0);
    private final CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
    private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);
    private final StatusSignal<Angle> pivotPosition = canCoder.getAbsolutePosition();

    /**
     * Algae Real constructor
     */
    public ElevatorAlgaeReal() {
        algaeMotorConfig.idleMode(IdleMode.kBrake);
        algaeMotorConfig.inverted(true);
        algaeMotor.configure(algaeMotorConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        pivotConfig.Slot0.kP = 0.0;
        pivotConfig.Slot0.kI = 0.0;
        pivotConfig.Slot0.kD = 0.0;
        pivotConfig.Slot0.kS = 0.0;
        pivotConfig.Slot0.kV = 0.0;
        pivotConfig.Slot0.kA = 0.0;
        pivotConfig.Slot0.kG = 0.0;
        pivotConfig.MotionMagic.MotionMagicAcceleration = 0.0;
        pivotConfig.MotionMagic.MotionMagicCruiseVelocity = 0.0;
        pivotConfig.MotionMagic.MotionMagicJerk = 0.0;
        canCoderConfig.MagnetSensor.MagnetOffset = 0.0;

        pivotMotor.getConfigurator().apply(pivotConfig);
        canCoder.getConfigurator().apply(canCoderConfig);
    }

    @Override
    public void updateInputs(AlgaeIOInputs inputs) { // update inputs to IO layer
        BaseStatusSignal.refreshAll(pivotPosition);
        inputs.algaeMotorCurrent = algaeMotor.getOutputCurrent();
        inputs.pivotPositon = pivotPosition.getValue();
    }

    @Override
    public void setAlgaeMotorVoltage(double voltage) { // set hardware speed
        algaeMotor.setVoltage(voltage);
    }

    @Override
    public void setPivotVoltage(double voltage) {
        pivotMotor.setVoltage(voltage);
    }

    public void setPosition(double position) {
        pivotMotor.setControl(motionMagic.withPosition(position));
    }

}
