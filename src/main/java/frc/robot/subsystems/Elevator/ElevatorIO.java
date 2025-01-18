package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public class ElevatorInputs {
        public boolean limitSwitch;
    }

    public default void updateInputs(ElevatorInputs inputs) {}


    public default void setVoltage(double volts) {}

    public default void setPositon(double position) {}

}
