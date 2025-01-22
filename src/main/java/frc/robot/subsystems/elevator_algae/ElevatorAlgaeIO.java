package frc.robot.subsystems.elevator_algae;

import org.littletonrobotics.junction.AutoLog;

/**
 * elevator algae io class
 */
public interface ElevatorAlgaeIO {
    /**
     * Elevator Algae inputs
     */
    @AutoLog
    public class AlgaeIOInputs {
        double AlgaeMotorSpeed;
    }

    public default void setAlgaeMotorSpeed(double speed) {}

    public default void updateInputs(AlgaeIOInputs inputs) {}
}
