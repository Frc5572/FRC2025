package frc.robot.subsystems.elevator_algae;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.Constants;

/**
 * Algae Real Class
 */
public class ElevatorAlgaeReal implements ElevatorAlgaeIO {
    private final SparkFlex algaeMotor = // Algae motor
        new SparkFlex(Constants.Motors.AlgaeMotors.ALGAE_MOTOR_ID, MotorType.kBrushless);
    private final SparkFlexConfig algaeMotorConfig = new SparkFlexConfig();

    /**
     * Algae Real constructor
     */
    public ElevatorAlgaeReal() {
        algaeMotorConfig.idleMode(IdleMode.kBrake);
        algaeMotor.configure(algaeMotorConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(AlgaeIOInputs inputs) { // update inputs to IO layer
        inputs.algaeMotorCurrent = algaeMotor.getOutputCurrent();
    }

    @Override
    public void setAlgaeMotorVoltage(double voltage) { // set hardware speed
        algaeMotor.setVoltage(voltage);
    }

}
