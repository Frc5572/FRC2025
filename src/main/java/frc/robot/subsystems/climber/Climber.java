package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;
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
        // Logger.processInputs("Climber", climberAutoLogged);

    }

    public void setClimberMotor(double percentage) {
        Logger.recordOutput("/Climber/Climber Percentage", percentage);
        io.setClimberMotorPercentage(percentage);
    }

    public boolean getClimberBeamBrakeStatus() {
        return climberAutoLogged.climberBeamBrake;
    }



    public Command runClimberMotor(double climberSpeed) {
        return Commands.startEnd(() -> {
            setClimberMotor(climberSpeed);

        }, () -> {
            setClimberMotor(50);

        }, this);
    }
}
