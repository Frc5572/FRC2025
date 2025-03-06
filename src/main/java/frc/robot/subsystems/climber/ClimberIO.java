package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Radians;
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
        public Angle climberPosition = Radians.of(0);
        public boolean climberTouchSensor;
        public boolean leftMagnet;
        public boolean rightMagnet;
    }

    public void updateInputs(ClimberInputs inputs);

    public void setClimbMotorVoltage(double voltage);

    public void setEncoderPoisiton(double position);

    /** Do nothing implementation */
    public static class Empty implements ClimberIO {

        @Override
        public void updateInputs(ClimberInputs inputs) {

        }

        @Override
        public void setClimbMotorVoltage(double voltage) {

        }

        @Override
        public void setEncoderPoisiton(double position) {

        }

    }
}
