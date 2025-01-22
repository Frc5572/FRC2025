package frc.robot.subsystems.PrimaryCoralScoring;

import org.littletonrobotics.junction.AutoLog;

public interface CoralScoringIO {

    @AutoLog

    public static class CoralScoringInputs {
        public double scoringRPM;
        public boolean scoringBeamBrake;

    }

    public default void updateInputs(CoralScoringInputs inputs) {}

    public default void setCoralScoringMotorPercentage(double percent) {}

}
