package frc.robot.subsystems.elevator_algae;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.Constants;

/**
 * Algae Real Class
 */
public class ElevatorAlgaeReal implements ElevatorAlgaeIO {
    private final SparkFlex AlgaeMotor = // Algae motor
        new SparkFlex(Constants.Motors.AlgaeMotors.INTAKE_MOTOR_ID, MotorType.kBrushless);
    private final SparkFlexConfig algaeMotorConfig = new SparkFlexConfig();

    public ElevatorAlgaeReal() {
        algaeMotorConfig.idleMode(IdleMode.kCoast);
        AlgaeMotor.configure(algaeMotorConfig,
            com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(AlgaeIOInputs inputs) { // update inputs to IO layer
        inputs.AlgaeMotorSpeed = AlgaeMotor.get();
    }

    @Override
    public void setAlgaeMotorVoltage(double voltage) { // set hardware speed
        AlgaeMotor.setVoltage(voltage);
    }

}
