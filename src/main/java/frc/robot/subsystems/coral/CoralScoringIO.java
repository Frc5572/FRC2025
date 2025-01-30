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
        public boolean grabingBeamBrakeRight;
        public boolean grabingBeamBrakeLeft;

    }

    public default void updateInputs(CoralScoringInputs inputs) {}

    public default void setCoralScoringMotorPercentage(double percent) {}

}
