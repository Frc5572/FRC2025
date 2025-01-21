package frc.robot.subsystems.climber;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.climber.ClimberIO.ClimberInputs;

public class ClimberReal implements ClimberIO {
    private final TalonFX climberMotorLeft = new TalonFX(1);
    private final TalonFX climberMotorRight = new TalonFX(2);
    public final RelativeEncoder climberRelativeEnc = climberMotorLeft.getEncoder();
    // private final TalonFX indexerMotor = new TalonFX(3);

    // private final DutyCycleOut indexerDutyCycleOut = new DutyCycleOut(0);
    // private final TalonFXConfiguration indexerConfig = new TalonFXConfiguration();
    // private final DigitalInput indexerBeamBrake = new DigitalInput(4);
    private final DigitalInput climberBeamBrake = new DigitalInput(5);
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    SparkMaxConfig rightConfig = new SparkMaxConfig();

    /**
     * climber IO Layer with real motors and sensors
     */
    public ClimberReal() {
        // SparkMaxConfig config = new SparkMaxConfig();
        // config.signals.primaryEncoderPositionPeriodMs(5);
        leftConfig.inverted(true).idleMode(IdleMode.kCoast).smartCurrentLimit(40)
            .voltageCompensation(12);
        rightConfig.inverted(false).idleMode(IdleMode.kCoast).smartCurrentLimit(40)
            .voltageCompensation(12);
        // indexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // indexerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        // indexerMotor.getConfigurator().apply(indexerConfig);
        climberMotorLeft.configure(leftConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
        climberMotorRight.configure(rightConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(ClimberInputs inputs) {
        // inputs.intakeSupplyVoltage = intakeMotorLeft.getBusVoltage();
        // inputs.intakeAmps = intakeMotorLeft.getOutputCurrent();
        // inputs.intakeRPM = intakeRelativeEnc.getVelocity();
        // inputs.indexerSupplyVoltage =
        // indexerMotor.getSupplyVoltage().getValueAsDouble();
        // inputs.indexerMotorVoltage =
        // indexerMotor.getMotorVoltage().getValueAsDouble();
        // inputs.indexerAmps = indexerMotor.getSupplyCurrent().getValueAsDouble();
        // inputs.indexerRPM = indexerMotor.getVelocity().getValueAsDouble();
        // inputs.indexerBeamBrake = !indexerBeamBrake.get(); // true == game piece
        inputs.climberBeamBrake = !climberBeamBrake.get(); // true == game piece
    }

    @Override
    public void setClimberMotorPercentage(double percent) {
        // Left ratio is 60:30
        // Right ratio is 32:30
        climberMotorLeft.set(percent);
        climberMotorRight.set(percent);
    }

    // @Override
    // public void setIndexerMotorPercentage(double percent) {
    // indexerMotor.setControl(indexerDutyCycleOut.withOutput(percent));
    // }

}
