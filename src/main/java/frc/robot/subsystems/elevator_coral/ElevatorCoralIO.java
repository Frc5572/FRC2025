package frc.robot.subsystems.elevator_coral;

import org.littletonrobotics.junction.AutoLog;

/**
 * IO layer for Elevator Coral subsystem
 */
public interface ElevatorCoralIO {
    /**
     * Elevator Coral Inputs
     */
    @AutoLog
    public class ElevatorCoralIOInputs {
        double feederMotorSpeed;

    }

    public default void updateInputs(ElevatorCoralIOInputs inputs) {}

    public default void setFeederMotorSpeed(double speed) {}

}
