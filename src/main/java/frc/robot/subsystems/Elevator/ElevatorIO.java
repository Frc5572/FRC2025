package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

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
        public Voltage outputVoltage;
        public Distance position;
        public AngularVelocity velocity;
    }

    public default void updateInputs(ElevatorInputs inputs) {}


    public default void setVoltage(double volts) {}

    public default void setPositon(double position) {}

}
