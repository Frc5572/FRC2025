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
        double algaeMotorCurrent;
        boolean beamBrakeStatus;
    }

    public default void setAlgaeMotorVoltage(double voltage) {}

    public default void updateInputs(AlgaeIOInputs inputs) {}

}
