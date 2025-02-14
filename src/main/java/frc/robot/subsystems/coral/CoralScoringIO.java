package frc.robot.subsystems.coral;

import org.littletonrobotics.junction.AutoLog;

/**
 * Coral Scoring IO
 */

public interface CoralScoringIO {

    /**
     * Getting coral inpusts
     */

    @AutoLog

    public static class CoralScoringInputs {
        public double scoringRPM;
        public boolean scoringBeamBrake;
        public boolean intakeBeamBrake;
    }

    public void updateInputs(CoralScoringInputs inputs);

    public void setCoralScoringMotorPercentage(double percent);

    /** Empty Coral Scoring implementation (for replay) */
    public static class Empty implements CoralScoringIO {

        @Override
        public void updateInputs(CoralScoringInputs inputs) {

        }

        @Override
        public void setCoralScoringMotorPercentage(double percent) {

        }

    }

}
