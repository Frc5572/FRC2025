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
    private final DigitalInput scoringBeamBrake =
        new DigitalInput(Constants.CoralScoringConstants.Scoring_Beam_Brake_DIO_Port);
    private final DigitalInput coralTouchSensor =
        new DigitalInput(Constants.CoralScoringConstants.Coral_Touch_Sensor_DIO_Port);
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    private final DigitalInput randomTouchSensor =
        new DigitalInput(Constants.CoralScoringConstants.Random_Touch_Sensor);

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
        inputs.randomTouchSensor = randomTouchSensor.get();
        inputs.scoringBeamBrake = !scoringBeamBrake.get();
        inputs.grabingBeamBrakeRight = coralTouchSensor.get();
    }

    public void setCoralScoringMotorPercentage(double percent) {
        coralScoringMotor.set(percent);
    }

}
