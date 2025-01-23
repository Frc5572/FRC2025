package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.Angle;

public interface ClimberIO {
    @AutoLog
    public static class ClimberInputs {
        public Angle climberPosition;
        public boolean climberTouchSensor;



    }

    public default void ClimberInputsAutoLogged(ClimberInputs inputs) {}

    public default void setClimbMotor(double power) {}


}
