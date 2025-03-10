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
    }

    public void setAlgaeMotorVoltage(double voltage);

    public void updateInputs(AlgaeIOInputs inputs);

    /** Empty Algae implementation (for replay) */
    public static class Empty implements ElevatorAlgaeIO {

        @Override
        public void setAlgaeMotorVoltage(double voltage) {

        }

        @Override
        public void updateInputs(AlgaeIOInputs inputs) {

        }

    }

}
