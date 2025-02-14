package frc.robot.subsystems.climber;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

/**
 * Climber Subsystem
 */
public class Climber extends SubsystemBase {
    private ClimberIO io;
    private ClimberInputsAutoLogged climberAutoLogged = new ClimberInputsAutoLogged();
    public Trigger resetButton = new Trigger(() -> getClimberTouchSensorStatus());

    public Climber(ClimberIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(climberAutoLogged);
        Logger.processInputs("Climber", climberAutoLogged);
    }

    public void setClimberMotorVoltage(double voltage) {
        Logger.recordOutput("/Climber/Climber Voltage", voltage);
        io.setClimbMotorVoltage(voltage);
    }

    public boolean getClimberTouchSensorStatus() {
        return climberAutoLogged.climberTouchSensor;
    }



    /**
     *
     * @return Set Motor Voltage until reached certain angle
     */
    public Command runClimberMotorCommand() { // run
        return Commands.runEnd(() -> setClimberMotorVoltage(Constants.Climb.VOLTAGE), () -> {
            setClimberMotorVoltage(0);
            System.out.println("Climber Done!");
        }, this).until(passedMaxAngle()).unless(passedMaxAngle());
    }

    /**
     *
     * @return Set Motor Voltage until reached certain angle
     */
    public Command runClimberMotorCommand(DoubleSupplier volts) { // run
        return Commands.runEnd(() -> setClimberMotorVoltage(volts.getAsDouble() * 3), () -> {
            setClimberMotorVoltage(0);
            System.out.println("Climber Done!");
        }, this).until(passedMaxAngle()).unless(passedMaxAngle());
    }


    /**
     *
     * @return Climber Position
     */
    public BooleanSupplier passedMaxAngle() { // degrees
        return () -> climberAutoLogged.climberPosition
            .baseUnitMagnitude() >= Constants.Climb.MAX_ANGLE.baseUnitMagnitude();

        // Constants.Climb.GEAR_RATIO >= Constants.Climb.MAX_ANGLE.in(Degrees);

    }

    /**
     *
     * @return Bring Climb Subsystem down until button is pressed.
     */
    public Command resetClimberCommand() { // reset
        return runEnd(() -> setClimberMotorVoltage(Constants.Climb.RESET_VOLTAGE),
            () -> setClimberMotorVoltage(0)).until(resetButton).unless(resetButton);
    }

    public Command restEncoder() {
        return Commands.runOnce(() -> io.setEncoderPoisiton(0.0)).ignoringDisable(true);
    }


}
// -250
