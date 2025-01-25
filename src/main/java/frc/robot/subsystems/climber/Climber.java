package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Degrees;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Climber Subsystem
 */
public class Climber extends SubsystemBase {
    private ClimberIO io;
    private ClimberInputsAutoLogged climberAutoLogged = new ClimberInputsAutoLogged();

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



    // rotations * gear ratio (1)
    public Command runClimberMotorCommand() { // run
        return Commands
            .runEnd(() -> setClimberMotorVoltage(Constants.Climb.VOLTAGE),
                () -> setClimberMotorVoltage(0), this)
            .until(passedMaxAngle()).unless(passedMaxAngle());
    }

    public BooleanSupplier passedMaxAngle() { // degrees
        return () -> climberAutoLogged.climberPosition.in(Degrees)
            * Constants.Climb.GEAR_RATIO >= Constants.Climb.MAX_ANGLE.in(Degrees);

    }


    public Command resetClimberCommand() { // reset
        return runEnd(() -> setClimberMotorVoltage(Constants.Climb.RESET_VOLTAGE),
            () -> setClimberMotorVoltage(0)).until(() -> getClimberTouchSensorStatus())
                .unless(() -> getClimberTouchSensorStatus());

    }
}
