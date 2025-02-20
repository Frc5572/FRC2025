package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Rotations;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;


/*** Class */
public class ClimberSim implements ClimberIO {
    // private final DCMotor climberMotors = DCMotor.getKrakenX60(2);
    private final DigitalInput climberTouchSensor =
        new DigitalInput(Constants.Climb.TOUCH_SENSOR_CHANNEL);
    private double offset = 0;
    private double currentPoint = 0;
    private double currentVoltage = 0;
    private final double distancePerVoltPerSecond = 10;


    /*** Real */
    public ClimberSim() {}


    @Override
    public void updateInputs(ClimberInputs inputs) {
        if ((currentPoint >= 0 && currentVoltage < 0)
            || (currentPoint <= Constants.Climb.MAX_ANGLE.in(Rotations) && currentVoltage > 0)) {
            currentPoint += currentVoltage * 0.02 * distancePerVoltPerSecond;
        }

        inputs.climberPosition =
            Rotations.of(MathUtil.clamp(currentPoint, 0, Constants.Climb.MAX_ANGLE.in(Rotations)));
        inputs.climberTouchSensor = currentPoint < 1;
    }


    @Override
    public void setClimbMotorVoltage(double voltage) {
        currentVoltage = voltage;
    }

    @Override
    public void setEncoderPoisiton(double position) {
        currentPoint = 0;
    }



}
