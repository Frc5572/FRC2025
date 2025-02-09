package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import edu.wpi.first.wpilibj.Timer;

public class ElevatorSim implements ElevatorIO {

    private final Timer timer = new Timer();
    private static final double timeToHeight = 1.5;

    private static final double m0 = 1.0;
    private static final double distancePerVoltPerSecond = 0.1;

    private double prevPoint;
    private double currentTarget;

    private double currentPoint;

    private boolean isPositionControl = false;
    private double currentVoltage = 0;

    private double offset = 0;

    public ElevatorSim() {
        timer.start();
        currentPoint = 0;
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        if (isPositionControl) {
            double t = timer.get();
            if (t >= timeToHeight) {
                t = timeToHeight;
            }
            t /= timeToHeight;
            currentPoint = (2 * t * t * t - 3 * t * t + 1) * prevPoint
                + (t * t * t - 2 * t * t + t) * m0 + (-2 * t * t * t + 3 * t * t) * currentTarget;
        } else {
            currentPoint += currentVoltage * 0.02 * distancePerVoltPerSecond;
        }

        inputs.position = Inches.of(currentPoint + offset);
        inputs.limitSwitch = currentPoint < 0.7;
    }

    @Override
    public void setVoltage(double volts) {
        isPositionControl = false;
    }

    @Override
    public void setPositon(double position) {
        timer.reset();
        prevPoint = currentPoint;
        currentTarget = position;
        isPositionControl = true;
    }

    @Override
    public void resetHome() {
        offset = -currentPoint;
    }

}
