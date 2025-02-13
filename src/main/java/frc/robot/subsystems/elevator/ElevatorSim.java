package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.Inches;
import edu.wpi.first.math.util.Units;

/** Simulator for Elevator */
public class ElevatorSim implements ElevatorIO {

    private static final double simkP = 19 / 48.0;
    private static final double distancePerVoltPerSecond = 12;

    private double currentTarget;

    private double currentPoint;

    private boolean isPositionControl = false;
    private double currentVoltage = 0;

    private double offset = 0;

    /** Simulator for Elevator */
    public ElevatorSim() {
        currentPoint = 0;
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        double currPos = currentPoint + offset;
        if (isPositionControl) {
            double err = (currentTarget - currPos);
            currentVoltage = err * simkP;
        }
        currentPoint += currentVoltage * 0.02 * distancePerVoltPerSecond;
        currPos = currentPoint + offset;

        inputs.position = Inches.of(currPos);
        inputs.limitSwitch = currentPoint < 0.7;
    }

    @Override
    public void setVoltage(double volts) {
        isPositionControl = false;
        currentVoltage = volts;
    }

    @Override
    public void setPositon(double position) {
        currentTarget = Units.metersToInches(position);
        isPositionControl = true;
    }

    @Override
    public void resetHome() {
        offset = -currentPoint;
    }

}
