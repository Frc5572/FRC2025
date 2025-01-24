package frc.robot.subsystems.coral;

import org.littletonrobotics.junction.AutoLog;

/**
 * Coral Scoring IO
 */

public interface CoralScoringIO {

    @AutoLog

    public static class CoralScoringInputs {
        public double scoringRPM;
        public boolean scoringBeamBrake;
        public boolean grabingBeamBrake;

    }

    public default void updateInputs(CoralScoringInputs inputs) {}

    public default void setCoralScoringMotorPercentage(double percent) {}

}
