package frc.robot.subsystems.ElevatorCoral;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorCoral extends SubsystemBase {
    ElevatorCoralIO io;
    ElevatorCoralIOInputsAutoLogged inputs = new ElevatorCoralIOInputsAutoLogged();

    /*
     * Constructor
     */
    public ElevatorCoral(ElevatorCoralIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }

    public void setFeederMotorSpeed(double speed) {
        io.setFeederMotorSpeed(speed);
    }

    public Command runFeederMotorCommand(double speed) {
        return Commands.runEnd(() -> io.setFeederMotorSpeed(speed), () -> io.setFeederMotorSpeed(0),
            this);
    }
}
