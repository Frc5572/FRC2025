package frc.robot.subsystems.elevator_algae;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Elevator Algae class
 */
public class ElevatorAlgae extends SubsystemBase {
    ElevatorAlgaeIO io;
    AlgaeIOInputsAutoLogged inputs = new AlgaeIOInputsAutoLogged();

    /*
     * Constructor for Elevator Algae class
     */
    public ElevatorAlgae(ElevatorAlgaeIO io) {
        this.io = io;
        io.updateInputs(inputs);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }


    public void setAlgaeMotorSpeed(double speed) { // set motor speed function
        io.setAlgaeMotorSpeed(speed);
    }

    public Command runMotorCommand(double speed) { // set motor speed Command
        return Commands.runEnd(() -> setAlgaeMotorSpeed(speed), () -> setAlgaeMotorSpeed(0), this);
    }

}
