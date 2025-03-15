package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Radians;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.LoggedTracer;
import frc.lib.util.viz.Viz2025;
import frc.robot.Constants;

/**
 * Climber Subsystem
 */
public class Climber extends SubsystemBase {
    private ClimberIO io;
    private ClimberInputsAutoLogged climberAutoLogged = new ClimberInputsAutoLogged();
    private final Viz2025 viz;
    public Trigger reachedClimberStart = new Trigger(() -> reachedClimberStart());


    public Climber(ClimberIO io, Viz2025 viz) {
        this.io = io;
        this.viz = viz;
    }

    @Override
    public void periodic() {
        io.updateInputs(climberAutoLogged);
        Logger.processInputs("Climber", climberAutoLogged);
        viz.setClimberAngle(climberAutoLogged.climberPosition);
        SmartDashboard.putBoolean("Climber Out", reachedClimberStart());
        LoggedTracer.record("Climber");
    }

    public void setClimberMotorVoltage(double voltage) {
        Logger.recordOutput("/Climber/Climber Voltage", voltage);
        io.setClimbMotorVoltage(voltage);
    }

    /**
     *
     * @return Set Motor Voltage until reached certain angle
     */
    public Command runClimberMotorCommand(double voltage, BooleanSupplier angle) { // run
        return Commands.runEnd(() -> setClimberMotorVoltage(voltage), () -> {
            setClimberMotorVoltage(0);
        }, this).until(angle).unless(angle);
    }

    /**
     *
     * @return Set Motor Voltage until reached certain angle
     */
    public Command runClimberMotorCommand(BooleanSupplier angle) { // run
        return Commands.runEnd(() -> setClimberMotorVoltage(Constants.Climb.CLIMB_VOLTAGE), () -> {
            setClimberMotorVoltage(0);
        }, this).until(angle).unless(angle);
    }

    /**
     *
     * @return Set Motor Voltage until reached certain angle
     */
    public Command manualClimb(DoubleSupplier volts) { // run
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
    }


    /**
     *
     * @return Climber Position
     */
    public BooleanSupplier passedClimbAngle() { // degrees
        return () -> climberAutoLogged.climberPosition
            .baseUnitMagnitude() >= Constants.Climb.CLIMB_ANGLE.baseUnitMagnitude();
    }

    /**
     *
     * @return Climber Position
     */
    public Boolean reachedClimberStart() { // degrees
        return climberAutoLogged.climberPosition
            .in(Radians) >= Constants.Climb.CLIMBER_START_ANGLE.in(Radians) - 20;
    }

    /**
     *
     * @return Bring Climb Subsystem down until button is pressed.
     */
    public Command resetClimberCommand() { // reset
        return runEnd(() -> setClimberMotorVoltage(Constants.Climb.RESET_VOLTAGE),
            () -> setClimberMotorVoltage(0));
    }

    public Command resetEncoder() {
        return Commands.runOnce(() -> io.setEncoderPoisiton(0.0)).ignoringDisable(true);
    }

    public Angle getClimberPosition() {
        return climberAutoLogged.climberPosition;
    }


}
// -250
