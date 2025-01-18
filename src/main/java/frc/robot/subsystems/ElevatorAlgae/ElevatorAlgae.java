package frc.robot.subsystems.ElevatorAlgae;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorAlgae extends SubsystemBase {
    ElevatorAlgaeIO io;
    ElevatorAlgaeIOInputsAutoLogged ElevatorAlgaeAutoLogged = new elevatorAlgaeIOInputsAutoLogged();

    public ElevatorAlgae(ElevatorAlgaeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(ElevatorAlgaeAutoLogged);
    }

    public void setAlgaeMotorSpeed(double speed) {
        io.setAlgaeMotorSpeed(speed);
    }

    public Command runMotorCommand(double speed) {
        Commands.runEnd(() -> setAlgaeMotorSpeed(speed), () -> setAlgaeMotorSpeed(0), this);
    }

}
