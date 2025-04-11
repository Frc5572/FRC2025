package frc.robot.subsystems.algaewrist;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;
import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface AlgaeWristIO {

    @AutoLog
    public static class AlgaeWristInputs {
        public Angle wristAngle = Degrees.of(0);
        public Voltage voltage = Volts.of(0);
        public Current current = Amps.of(0);
    }

    public void updateInputs(AlgaeWristInputs inputs);

    public void setVoltage(double voltage);

    public void setWristSetpoint(Angle angle);

    public static class Empty implements AlgaeWristIO {

        @Override
        public void updateInputs(AlgaeWristInputs inputs) {}

        @Override
        public void setWristSetpoint(Angle angle) {}

        @Override
        public void setVoltage(double volts) {}
    }
}
