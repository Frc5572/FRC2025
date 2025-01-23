package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Climber Subsystem
 */
public class Climber extends SubsystemBase {
    private ClimberIO io;
    private ClimberIO.ClimberInputs climberAutoLogged = new ClimberIO.ClimberInputs();

    public Climber(ClimberIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.ClimberInputsAutoLogged(climberAutoLogged);
        Logger.processInputs("Climber", (LoggableInputs) climberAutoLogged);

    }

    public void setClimberMotor(double power) {
        Logger.recordOutput("/Climber/Climber Percentage", power);
        io.setClimbMotor(power);
    }

    public boolean getClimberTouchSensorStatus() {
        return climberAutoLogged.climberTouchSensor;
    }



    public Command runClimberMotor() { 
        return Commands.startEnd(() ->
            setClimberMotor(climberSpeed.getAsDouble() * 12);


        }, () -> {
            setClimberMotor(0);

        }, this);
    }
}
