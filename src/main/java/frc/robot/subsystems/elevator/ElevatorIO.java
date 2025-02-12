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

    public void updateInputs(ElevatorInputs inputs);

    public void setVoltage(double volts);

    public void setPositon(double position);

    public void resetHome();

    /** Empty Elevator implementation (for replay) */
    public static class Empty implements ElevatorIO {

        @Override
        public void updateInputs(ElevatorInputs inputs) {

        }

        @Override
        public void setVoltage(double volts) {

        }

        @Override
        public void setPositon(double position) {

        }

        @Override
        public void resetHome() {

        }

    }

    public default void setPower(double power) {}

}
