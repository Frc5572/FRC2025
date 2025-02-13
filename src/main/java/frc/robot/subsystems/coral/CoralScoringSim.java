package frc.robot.subsystems.coral;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

/**
 * Real Class Coral Scoring
 */

public class CoralScoringSim implements CoralScoringIO {
    private final DigitalInput scoringBeamBrake =
        new DigitalInput(Constants.CoralScoringConstants.Scoring_Beam_Brake_DIO_Port);
    private final DigitalInput coralTouchSensor =
        new DigitalInput(Constants.CoralScoringConstants.Coral_Touch_Sensor_DIO_Port);

    /**
     * Coral Scoring Real
     */
    public CoralScoringSim() {}

    /**
     * updating coral beam brakes
     */

    public void updateInputs(CoralScoringInputs inputs) {
        inputs.scoringBeamBrake = !scoringBeamBrake.get();
        inputs.grabingBeamBrakeRight = coralTouchSensor.get();
    }

    public void setCoralScoringMotorPercentage(double percent) {}

}
