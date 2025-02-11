package frc.robot.subsystems.elevator_algae;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.viz.Viz2025;
import frc.robot.Constants;

/**
 * Elevator Algae class
 */
public class ElevatorAlgae extends SubsystemBase {
    ElevatorAlgaeIO io;
    AlgaeIOInputsAutoLogged inputs = new AlgaeIOInputsAutoLogged();
    private final Viz2025 viz;

    /**
     * Constructor for Elevator Algae class
     */
    public ElevatorAlgae(ElevatorAlgaeIO io, Viz2025 viz) {
        this.viz = viz;
        this.io = io;
        io.updateInputs(inputs);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Algae", inputs);
        viz.setHasAlgae(hasAlgae());
    }


    private void setAlgaeMotorVoltage(double voltage) { // set motor speed function
        io.setAlgaeMotorVoltage(voltage);
    }

    /** Get if we're holding algae */
    public boolean hasAlgae() {
        return inputs.algaeMotorCurrent > Constants.HAS_ALGAE_CURRENT_THRESHOLD;
    }

    public Command setMotorVoltageCommand(double speed) { // set motor speed Command
        return runEnd(() -> setAlgaeMotorVoltage(speed), () -> setAlgaeMotorVoltage(0));
        // .until(() -> hasAlgae());
    }
}
