package frc.robot.subsystems.elevator_algae;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.LoggedTracer;
import frc.lib.util.viz.Viz2025;
import frc.robot.Constants;

/**
 * Elevator Algae class
 */
public class ElevatorAlgae extends SubsystemBase {
    ElevatorAlgaeIO io;
    AlgaeIOInputsAutoLogged inputs = new AlgaeIOInputsAutoLogged();
    private final Viz2025 viz;

    /** Get if algae is held */
    public Trigger hasAlgae =
        new Trigger(() -> inputs.algaeMotorCurrent > Constants.Algae.HAS_ALGAE_CURRENT_THRESHOLD)
            .debounce(.25);

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
        viz.setHasAlgae(hasAlgae.getAsBoolean());
        Color temp = Color.kBlack;
        if (hasAlgae.getAsBoolean()) {
            temp = Color.kGreen;
        } else if (!hasAlgae.getAsBoolean()) {
            temp = Color.kRed;
        }
        SmartDashboard.putString("Dashboard/Main Driver/Have Algae", temp.toHexString());
        LoggedTracer.record("Algae");
    }


    private void setAlgaeMotorVoltage(double voltage) { // set motor speed function
        Logger.recordOutput("Algae/Voltage", voltage);
        io.setAlgaeMotorVoltage(voltage);
    }

    // /** Get if we're holding algae */
    // public boolean hasAlgae() {
    // return inputs.algaeMotorCurrent > Constants.Algae.HAS_ALGAE_CURRENT_THRESHOLD;
    // }

    /** Run algae intake with given speed */
    public Command runAlgaeMotor(double speed) { // set motor speed Command
        return runEnd(() -> setAlgaeMotorVoltage(speed), () -> setAlgaeMotorVoltage(0));
        // .until(() -> hasAlgae());
    }

    /** Run algae intake with given speed */
    public Command runAlgaeMotor(DoubleSupplier speed) { // set motor speed Command
        return runEnd(() -> setAlgaeMotorVoltage(speed.getAsDouble()),
            () -> setAlgaeMotorVoltage(0));
        // .until(() -> hasAlgae());
    }

    /**
     * Keeps algae intake motor running even after it has intaked an algae, but it lowers the speed
     */
    public Command algaeIntakeCommand() {
        return runAlgaeMotor(Constants.Algae.VOLTAGE).until(hasAlgae)
            .andThen(() -> setAlgaeMotorVoltage(Constants.Algae.SMALLER_VOLTAGE));

    }

    /**
     * Keeps algae intake motor running even after it has intaked an algae, but it lowers the speed
     */
    public Command algaeIntakeCommand(BooleanSupplier supplier) {
        return runAlgaeMotor(() -> supplier.getAsBoolean() ? Constants.Algae.VOLTAGE : 0)
            .until(hasAlgae).andThen(runAlgaeMotor(Constants.Algae.SMALLER_VOLTAGE)
                .until(() -> !supplier.getAsBoolean()))
            .repeatedly();
    }
}
