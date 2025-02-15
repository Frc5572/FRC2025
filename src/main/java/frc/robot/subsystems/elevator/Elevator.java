package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.viz.Viz2025;
import frc.robot.Constants;

/**
 * Elevator Subsystem
 */
public class Elevator extends SubsystemBase {
    ElevatorIO io;
    private final Viz2025 viz;

    private ElevatorInputsAutoLogged inputs = new ElevatorInputsAutoLogged();

    /** Elevator Subsystem */
    public Elevator(ElevatorIO io, Viz2025 viz) {
        this.viz = viz;
        this.io = io;
        io.updateInputs(inputs);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
        viz.setElevatorHeight(inputs.position);
        Logger.recordOutput("Elevator/position_in", inputs.position.in(Inches));
        if (inputs.limitSwitch) {
            io.resetHome();
        }
    }

    /**
     * moves elevator to home
     *
     * @return elevator at home
     *
     */
    public Command home() {
        Command slowLower = Commands.runEnd(() -> io.setVoltage(-0.7), () -> io.setVoltage(0.0));
        return moveTo(() -> Constants.Elevator.HOME).until(() -> inputs.position.in(Inches) < 5.0)
            .andThen(slowLower).until(() -> (inputs.limitSwitch == true)).alongWith(
                Commands.runOnce(() -> Logger.recordOutput(Constants.Elevator.heightName, "home")));
    }

    /**
     * moves elevator to l2
     *
     * @return elevator at l2
     *
     */
    public Command p0() {
        return moveTo(() -> Constants.Elevator.P0);
    }

    public Command p1() {
        return moveTo(() -> Constants.Elevator.P1);
    }

    public Command p2() {
        return moveTo(() -> Constants.Elevator.P2);
    }

    public Command p3() {
        return moveTo(() -> Constants.Elevator.P3);
    }

    public Command p4() {
        return moveTo(() -> Constants.Elevator.P4);
    }

    public boolean hightNotHome() {
        return (inputs.position).in(Inches) >= (Constants.Elevator.P1).in(Inches);
    }

    /**
     * sets height of elevator
     *
     * @param height desired height of elevator
     * @return elevator height change
     *
     */
    public Command moveTo(Supplier<Distance> height) {
        return run(() -> {
            Logger.recordOutput("targetHeight", height.get().in(Meters));
            io.setPositon(height.get().in(Meters));
        }).until(() -> Math.abs(inputs.position.in(Inches) - height.get().in(Inches)) < 1);
    }

    public Command moveUp() {
        return runEnd(() -> io.setVoltage(SmartDashboard.getNumber("elevatorVoltage", 1.0)),
            () -> io.setVoltage(0));
    }

    public Command moveDown() {
        return runEnd(() -> io.setVoltage(-1.0), () -> io.setVoltage(0));
    }

    public Command moveToMagic(Supplier<Distance> height) {
        return run(() -> {
            Logger.recordOutput("targetHeight", height.get().in(Meters));
            io.setPositon(height.get().in(Meters));
        }).until(() -> Math.abs(inputs.position.in(Inches) - height.get().in(Inches)) < 1);
    }
}
