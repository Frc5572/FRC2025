package frc.robot.subsystems.climber;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
// import frc.lib.math.Conversions;
import frc.robot.Constants;



public class ClimberReal implements ClimberIO { // test
    private final TalonFX climberMotorLeft = new TalonFX(Constants.Climb.LeftTalonFXID, "canivore"); // Kraken
    private final TalonFX climberMotorRight =
        new TalonFX(Constants.Climb.RightTalonFXID, "canivore"); // Kraken Motor
    private final TalonFXConfiguration rightConfig = new TalonFXConfiguration(); // Kraken Config
    private final TalonFXConfiguration leftConfig = new TalonFXConfiguration(); // Kraken Config
    private StatusSignal<Angle> climbMotorSelectedPosition = climberMotorLeft.getPosition();
    // private final DigitalInput climberTouchSensor = new DigitalInput(5);
    new DigitalInput(Constants.Climb.TouchSenorChannel); // Touch Sensor


    public ClimberReal() { // test

        configClimberMotor(); // just do climberMotorLeft.get(_____), this is the encoder.
    }


    public void configClimberMotor() { // test
        leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climberMotorLeft.getConfigurator().apply(leftConfig);
        // climberMotorLeft.getConfigurator().setPosition(0.0);
        climberMotorRight.getConfigurator().apply(rightConfig);
        // climberMotorRight.getConfigurator().setPosition(0.0);
        climberMotorLeft.setControl(new Follower(climberMotorRight.getDeviceID(), true));
    }

    public void setClimbMotorVoltage(double pos) { // test
        // climberMotorRight.climbMotorSelectedPosition
        climberMotorRight.setVoltage(pos); // sets the velocity

    }

    public void sgetClimbMotorPosition() { // test

    }
}
