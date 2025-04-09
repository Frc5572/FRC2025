package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;


/*** Class */
public class ClimberReal implements ClimberIO {
    private final TalonFX climberMotorRight = new TalonFX(Constants.Climb.RIGHT_TALON_FX_ID, "rio");
    private final TalonFXConfiguration rightConfig = new TalonFXConfiguration();
    private StatusSignal<Angle> climbMotorPosition = climberMotorRight.getPosition();

    /*** Real */
    public ClimberReal() {
        rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // rightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        // leftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        // climberMotorLeft.getConfigurator().apply(leftConfig);
        climberMotorRight.getConfigurator().apply(rightConfig);


    }


    @Override
    public void updateInputs(ClimberInputs inputs) {
        BaseStatusSignal.refreshAll(climbMotorPosition);
        inputs.climberPosition = climbMotorPosition.getValue();
    }


    @Override
    public void setClimbMotorVoltage(double voltage) {
        climberMotorRight.setVoltage(voltage);
    }

    @Override
    public void setEncoderPoisiton(double position) {
        climberMotorRight.getConfigurator().setPosition(position);
    }



}
