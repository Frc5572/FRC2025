package frc.robot.subsystems.coral;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

/**
 * Real Class Coral Scoring
 */

public class CoralScoringSim implements CoralScoringIO {
    private final DigitalInput outtakeBeamBreak =
        new DigitalInput(Constants.CoralScoringConstants.OUTTAKE_BEAM_BREAK_DIO_PORT);
    private final DigitalInput intakeBeamBreak =
        new DigitalInput(Constants.CoralScoringConstants.INTAKE_BEAM_BREAK_DIO_PORT);

    /**
     * Coral Scoring Real
     */
    public CoralScoringSim() {}

    /**
     * updating coral beam brakes
     */

    public void updateInputs(CoralScoringInputs inputs) {
        inputs.outtakeBeamBreak = !outtakeBeamBreak.get();
        inputs.intakeBeamBreak = !intakeBeamBreak.get();
    }

    public void setCoralPower(double percent) {}

}
