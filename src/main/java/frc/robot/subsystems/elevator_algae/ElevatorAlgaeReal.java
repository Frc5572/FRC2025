package frc.robot.subsystems.elevator_algae;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
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
    private final SparkFlex algaeMotor = // Algae motor
        new SparkFlex(Constants.Motors.AlgaeMotors.ALGAE_MOTOR_ID, MotorType.kBrushless);
    private final SparkFlexConfig algaeMotorConfig = new SparkFlexConfig();
    private final CANcoder canCoder = new CANcoder(0);
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
    }

    @Override
    public void updateInputs(AlgaeIOInputs inputs) { // update inputs to IO layer
        BaseStatusSignal.refreshAll(pivotPosition);
        inputs.algaeMotorCurrent = algaeMotor.getOutputCurrent();
    }

    @Override
    public void setAlgaeMotorVoltage(double voltage) { // set hardware speed
        algaeMotor.setVoltage(voltage);
    }
}
