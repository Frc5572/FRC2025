package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/**
 * Elevator IO Class for Elevator
 */
public interface ElevatorIO {

    /**
     * Inputs Class for Elevator
     */

    @AutoLog
    public class ElevatorInputs {
        public boolean limitSwitch;
        public boolean atPositon;
        public Angle position;
        public AngularVelocity velocity;
    }

    public default void updateInputs(ElevatorInputs inputs) {}


    public default void setVoltage(double volts) {}

    public default void setPositon(double position) {}

}
