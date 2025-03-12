package frc.robot.subsystems.elevator_algae;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.AngularVelocity;

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
        AngularVelocity motorRPM;
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
