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
import frc.lib.util.ScoringLocation.Height;
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
            .andThen(slowLower).until(() -> (inputs.limitSwitch == true));
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

    public Command modeSwapper() {
        Command positon;
        switch (Height.getCurrentState()) {
            case kHome:
                SmartDashboard.putData("home", null);
                positon = home();
                break;
            case KPosition0:
                positon = p0();
                break;
            case KPosition1:
                positon = p1();
                break;
            case KPosition2:
                positon = p2();
                break;
            case KPosition3:
                positon = p3();
                break;
            case kPosition4:
                positon = p4();
                break;
            default:
                positon = home();
                break;

        }
        return positon;
    }
}
