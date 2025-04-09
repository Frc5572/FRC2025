package frc.robot.subsystems.elevator_algae;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.Angle;
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
        AngularVelocity motorRPM = RPM.of(0);
        Angle pivotPositon = Degrees.of(0.0);
    }

    public void setAlgaeMotorVoltage(double voltage);

    public void updateInputs(AlgaeIOInputs inputs);

    public void setPivotVoltage(double voltage);

    public void setPosition(double position);

    /** Empty Algae implementation (for replay) */
    public static class Empty implements ElevatorAlgaeIO {

        @Override
        public void setAlgaeMotorVoltage(double voltage) {

        }

        @Override
        public void updateInputs(AlgaeIOInputs inputs) {

        }

        @Override
        public void setPivotVoltage(double voltage) {}

        @Override
        public void setPosition(double position) {}


    }

}
