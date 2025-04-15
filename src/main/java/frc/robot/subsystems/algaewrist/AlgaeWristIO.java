package frc.robot.subsystems.algaewrist;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;
import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/** IO interface for algae wrist */
public interface AlgaeWristIO {

    /** Inputs for algae wrist */
    @AutoLog
    public static class AlgaeWristInputs {
        public Angle wristAngle = Degrees.of(0);
        public Voltage voltage = Volts.of(0);
        public Current current = Amps.of(0);
    }

    /** Update the inputs */
    public void updateInputs(AlgaeWristInputs inputs);

    /** Set desired angle */
    public void setWristSetpoint(Angle angle);

    /** Set desired voltage */
    public void setWristVoltage(double volts);

    /** Enable/disable brake mode */
    public void setBrakeMode(boolean brake);

    /** Empty IO for replay */
    public static class Empty implements AlgaeWristIO {

        @Override
        public void updateInputs(AlgaeWristInputs inputs) {}

        @Override
        public void setWristSetpoint(Angle angle) {}

        @Override
        public void setWristVoltage(double volts) {}

        @Override
        public void setBrakeMode(boolean brake) {}
    }
}

