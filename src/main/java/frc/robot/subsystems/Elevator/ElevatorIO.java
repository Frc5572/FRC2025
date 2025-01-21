package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

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
    }

    public default void updateInputs(ElevatorInputs inputs) {}


    public default void setVoltage(double volts) {}

    public default void setPositon(double position) {}

}
