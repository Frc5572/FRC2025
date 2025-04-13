package frc.robot.subsystems.algaewrist;

import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.units.measure.Angle;

public class AlgaeWristSim implements AlgaeWristIO {

    Angle angle = Degrees.of(45);

    public AlgaeWristSim() {

    }

    @Override
    public void updateInputs(AlgaeWristInputs inputs) {
        inputs.wristAngle = angle;
    }

    @Override
    public void setWristSetpoint(Angle angle) {
        this.angle = angle;
    }

    @Override
    public void setWristVoltage(double volts) {

    }

    @Override
    public void setBrakeMode(boolean brake) {
    }
}

