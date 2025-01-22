package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;


public interface ClimberIO {
    @AutoLog
    public static class ClimberInputs {
        public double climberRPM;
        public double indexerRPM;
        public boolean climberTouchSensor;



    }

    public default void ClimberInputsAutoLogged(ClimberInputs inputs) {}

    public default void setClimberMotorPercentage(double percent) {}


}
