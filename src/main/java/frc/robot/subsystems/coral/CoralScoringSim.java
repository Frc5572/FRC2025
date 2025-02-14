package frc.robot.subsystems.coral;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

/**
 * Real Class Coral Scoring
 */

public class CoralScoringSim implements CoralScoringIO {
    private final DigitalInput scoringBeamBrake =
        new DigitalInput(Constants.CoralScoringConstants.Scoring_Beam_Brake_DIO_Port);
    private final DigitalInput intakeBeamBrake =
        new DigitalInput(Constants.CoralScoringConstants.Intake_Beam_Brake_DIO_Port);

    /**
     * Coral Scoring Real
     */
    public CoralScoringSim() {}

    /**
     * updating coral beam brakes
     */

    public void updateInputs(CoralScoringInputs inputs) {
        inputs.scoringBeamBrake = !scoringBeamBrake.get();
        inputs.intakeBeamBrake = !intakeBeamBrake.get();
    }

    public void setCoralScoringMotorPercentage(double percent) {}

}
