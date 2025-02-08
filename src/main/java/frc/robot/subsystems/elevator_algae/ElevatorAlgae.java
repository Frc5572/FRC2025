package frc.robot.subsystems.elevator_algae;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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
        Logger.processInputs("Algae", inputs);
    }


    public void setAlgaeMotorVoltage(double voltage) { // set motor speed function
        io.setAlgaeMotorVoltage(voltage);
    }

    public boolean hasAlgae() {
        return inputs.algaeMotorCurrent > Constants.HAS_ALGAE_CURRENT_THRESHOLD;
    }

    public Command setMotorVoltageCommand(double speed) { // set motor speed Command
        return runEnd(() -> setAlgaeMotorVoltage(speed), () -> setAlgaeMotorVoltage(0));
        // .until(() -> hasAlgae());
    }
}
