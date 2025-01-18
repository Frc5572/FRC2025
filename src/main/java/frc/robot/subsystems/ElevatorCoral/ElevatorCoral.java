package frc.robot.subsystems.ElevatorCoral;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorCoral extends SubsystemBase {
    ElevatorCoralIO io;
    ElevatorAlgaeIOInputsAutoLogged ElevatorAlgaeAutoLogged = new ElevatorAlgaeIOInputsAutoLogged();

    /*
     * Constructor
     */
    public ElevatorCoral(ElevatorCoralIO io) {
        this.io = io;
    }

    public void setFeederMotorSpeed(double speed) {
        io.setFeederMotorSpeed(speed);
    }

    public Command runFeederMotorCommand(double speed) {
        Commands.runEnd(() -> io.setFeederMotorSpeed(speed), () -> io.setFeederMotorSpeed(0), this);
    }
}
