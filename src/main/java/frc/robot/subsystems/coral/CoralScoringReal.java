package frc.robot.subsystems.coral;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

/**
 * Real Class Coral Scoring
 */

public class CoralScoringReal implements CoralScoringIO {
    private final SparkMax coralScoringMotor = new SparkMax(
        Constants.Motors.PrimaryCoralScoring.Coral_Scoring_NEO_ID, MotorType.kBrushless);
    public final RelativeEncoder coralScoringRelativeEnc = coralScoringMotor.getEncoder();
    private final DigitalInput outtakeBeamBreak =
        new DigitalInput(Constants.CoralScoringConstants.OUTTAKE_BEAM_BREAK_DIO_PORT);
    private final DigitalInput intakeBeamBreak =
        new DigitalInput(Constants.CoralScoringConstants.INTAKE_BEAM_BREAK_DIO_PORT);
    SparkMaxConfig motorConfig = new SparkMaxConfig();

    /**
     * Coral Scoring Real
     */
    public CoralScoringReal() {
        motorConfig.idleMode(IdleMode.kBrake);
        coralScoringMotor.configure(motorConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
    }

    /**
     * updating coral beam brakes
     */

    public void updateInputs(CoralScoringInputs inputs) {
        inputs.outtakeBeamBreak = !outtakeBeamBreak.get();
        inputs.intakeBeamBreak = !intakeBeamBreak.get();
    }

    public void setCoralPower(double percent) {
        coralScoringMotor.set(percent);
    }

}
