package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public class ClimberIO {


    /**
     * Intake Inputs to Log
     */
    @AutoLog
    public static class ClimberInputs {

        public double climberRPM;
        public double indexerRPM;
        public boolean indexerBeamBrake;
        public boolean intakeBeamBrake;
    }

    public default void updateInputs() {}

    public default void setClimberMotorPercentage(double percent) {}

    public default void setIndexerMotorPercentage(double percent) {}

}
