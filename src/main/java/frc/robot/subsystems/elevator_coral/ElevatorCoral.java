package frc.robot.subsystems.elevator_coral;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
 * Elevator Coral main class
 */
public class ElevatorCoral extends SubsystemBase {
    ElevatorCoralIO io;
    ElevatorCoralIOInputsAutoLogged inputs = new ElevatorCoralIOInputsAutoLogged();

    /*
     * Constructor for Elevator Coral class
     */
    public ElevatorCoral(ElevatorCoralIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }

    /*
     * Set motor speed
     */
    public void setFeederMotorSpeed(double speed) {
        io.setFeederMotorSpeed(speed);
    }

    /*
     * Set motor speed command
     */
    public Command runFeederMotorCommand(double speed) {
        return Commands.runEnd(() -> io.setFeederMotorSpeed(speed), () -> io.setFeederMotorSpeed(0),
            this);
    }
}
