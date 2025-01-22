package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;



public class ClimberReal implements ClimberIO {
    private final TalonFX climberMotorLeft = new TalonFX(1, "Left"); // Kraken Motor
    private final TalonFX climberMotorRight = new TalonFX(2, "Right");// Kraken Motor
    private final TalonFXConfiguration rightConfig = new TalonFXConfiguration();// Kraken Motor
                                                                                // Configuration
    private final TalonFXConfiguration leftConfig = new TalonFXConfiguration();// Kraken Motor
                                                                               // Configration
    private final DigitalInput climberTouchSensor = new DigitalInput(5); // Touch Sensor
    private final VelocityVoltage climberVelocity = new VelocityVoltage(0); // velocity

    public ClimberReal() {
        leftConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        leftConfig.CurrentLimits.SupplyCurrentLimit = 40;
        rightConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rightConfig.CurrentLimits.SupplyCurrentLimit = 40;
        climberTouchSensor.get();
    }

    public void setClimbMotor() {
        climberMotorLeft.setControl(climberVelocity);
        climberMotorRight.setControl(climberVelocity);

    }
}
