package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.Angle;

/**
 * Climber
 */
public interface ClimberIO {
    /***
     * AutoLog
     */
    @AutoLog
    public static class ClimberInputs {
        public Angle climberPosition;
        public boolean climberTouchSensor;



    }

    public void updateInputs(ClimberInputs inputs);

    public void setClimbMotorVoltage(double voltage);


}
