package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
// import frc.lib.math.Conversions;
import frc.robot.Constants;



public class ClimberReal implements ClimberIO { // test
    private final TalonFX climberMotorLeft = new TalonFX(Constants.Climb.LeftTalonFXID, "canivore"); // Kraken
    private final TalonFX climberMotorRight =
        new TalonFX(Constants.Climb.RightTalonFXID, "canivore"); // Kraken Motor
    private final TalonFXConfiguration rightConfig = new TalonFXConfiguration(); // Kraken Config
    private final TalonFXConfiguration leftConfig = new TalonFXConfiguration(); // Kraken Config
    private final DigitalInput climberTouchSensor =
        new DigitalInput(Constants.Climb.TouchSenorChannel); // Touch Sensor

    // private final VelocityVoltage climberVoltage = new VelocityVoltage(0); // velocity



    public ClimberReal() { // test

        configClimberMotor(); // just do climberMotorLeft.get(_____), this is the encoder.
    }



    public void configClimberMotor() { // test
        // leftConfig.CurrentLimits.SupplyCurrentLimitEnable =
        // Constants.Climb.leftConfigSupplyCurrentLimitEnable;
        // leftConfig.CurrentLimits.SupplyCurrentLimit = Constants.Climb.leftSupplyCurrentLimit;
        // rightConfig.CurrentLimits.SupplyCurrentLimitEnable =
        // Constants.Climb.rightConfigSupplyCurrentLimitEnable;
        // rightConfig.CurrentLimits.SupplyCurrentLimit = Constants.Climb.rightSupplyCurrentLimit;
        leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climberMotorLeft.getConfigurator().apply(leftConfig);
        climberMotorLeft.getConfigurator().setPosition(0.0);
        climberMotorRight.getConfigurator().apply(rightConfig);
        climberMotorRight.getConfigurator().setPosition(0.0);
    }

    public void setClimbMotorVoltage(double pos) { // test
        // climberMotorLeft.setControl(climberVoltage pos); // sets the velocity
        climberMotorRight.setVoltage(pos); // sets the velocity

    }
}
