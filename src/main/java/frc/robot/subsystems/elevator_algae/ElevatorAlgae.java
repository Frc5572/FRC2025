package frc.robot.subsystems.elevator_algae;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
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

    /*
     * Set motor speed
     */
    public void setAlgaeMotorSpeed(double speed) {
        io.setAlgaeMotorSpeed(speed);
    }

    /*
     * Set motor speed command
     */
    public Command runMotorCommand(double speed) {
        return Commands.runEnd(() -> setAlgaeMotorSpeed(speed), () -> setAlgaeMotorSpeed(0), this);
    }

}
