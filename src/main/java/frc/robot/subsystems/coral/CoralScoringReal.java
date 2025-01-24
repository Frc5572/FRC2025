package frc.robot.subsystems.coral;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
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
    private final DigitalInput scoringBeamBrake =
        new DigitalInput(Constants.CoralScoringConstants.Scoring_Beam_Brake_DIO_Port);
    private final DigitalInput grabingBeamBrake =
        new DigitalInput(Constants.CoralScoringConstants.Grabing_Beam_Brake_DIO_Port);
    SparkMaxConfig motorConfig = new SparkMaxConfig();


    public CoralScoringReal() {
        coralScoringMotor.configure(motorConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
    }

    public void updateInputs(CoralScoringInputs inputs) {
        inputs.scoringBeamBrake = !scoringBeamBrake.get();
        inputs.grabingBeamBrake = !grabingBeamBrake.get();
    }

    public void setCoralScoringMotorPercentage(double percent) {
        coralScoringMotor.set(percent);
    }

}
