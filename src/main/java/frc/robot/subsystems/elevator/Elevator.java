package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Elevator Subsystem
 */
public class Elevator extends SubsystemBase {
    ElevatorIO io;
    Timer time = new Timer();
    private ElevatorInputsAutoLogged inputs = new ElevatorInputsAutoLogged();

    public Elevator(ElevatorIO io) {
        this.io = io;
        io.updateInputs(inputs);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }

    /**
     * moves elevator to home with time out
     *
     * @return home position
     */
    public Command homeTimer() {
        Command slowLower = Commands.run(() -> io.setVoltage(-0.1)).withTimeout(1);
        return moveTo(() -> Constants.Elevator.HOME).andThen(slowLower);
    }

    /**
     * moves elevator to home
     *
     * @return elevator at home
     * 
     */
    public Command home() {

        Command slowLower = Commands.run(() -> io.setVoltage(-0.1));
        return moveTo(() -> Constants.Elevator.HOME).andThen(slowLower)
            .until(() -> (inputs.limitSwitch == false));
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
}
