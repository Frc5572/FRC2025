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
    }

    /**
     * moves elevator to home with time out
     *
     * @return home position
     */
    public Command homeTimer() {
        Command slowLower = Commands.run(() -> io.setVoltage(-0.1));
        return moveTo(Constants.Elevator.HOME).andThen(slowLower).withTimeout(1);
    }

    /**
     * moves elevator to home
     *
     * @return elevator at home
     *
     */
    public Command home() {

        Command slowLower = Commands.run(() -> io.setVoltage(-0.1));
        return moveTo(Constants.Elevator.HOME).andThen(slowLower)
            .until(() -> (inputs.limitSwitch == true));
    }

    /**
     * moves elevator to l2
     *
     * @return elevator at l2
     *
     */
    public Command p0() {
        return moveTo(Constants.Elevator.P0);
    }

    public Command p1() {
        return moveTo(Constants.Elevator.P1);
    }

    public Command p2() {
        return moveTo(Constants.Elevator.P2);
    }

    public Command p3() {
        return moveTo(Constants.Elevator.P3);
    }

    public Command p4() {
        return moveTo(Constants.Elevator.P4);
    }

    public Command goToHeight(Supplier<Distance> height) {
        return run(() -> {
            double desiredHeight = height.get().in(Meters);
            Logger.recordOutput("targetHeight", desiredHeight);
            io.setPositon(desiredHeight);
        }).until(() -> Math.abs(inputs.position.in(Inches) - height.get().in(Inches)) < 1);
    }

    /**
     * sets height of elevator
     *
     * @param height desired height of elevator
     * @return elevator height change
     *
     */
    public Command moveTo(Distance height) {
        return run(() -> {
            Logger.recordOutput("targetHeight", height.in(Meters));
            io.setPositon(height.in(Meters));
        }).until(() -> Math.abs(inputs.position.in(Inches) - height.in(Inches)) < 1);
    }

    public Command moveUp() {
        return runEnd(() -> io.setVoltage(SmartDashboard.getNumber("elevatorVoltage", 1.0)),
            () -> io.setVoltage(0));
    }

    public Command moveDown() {
        return runEnd(() -> io.setVoltage(-1.0), () -> io.setVoltage(0));
    }

    public Command modeSwapper() {
        switch (Height.getCurrentState()) {
            case kHome:
                return home();
            case KPosition0:
                return p0();
            case KPosition1:
                return p1();
            case KPosition2:
                return p2();
            case KPosition3:
                return p3();
            case kPosition4:
                return p4();
            default:
                return home();
        }
        // switch (HeightMode.getCurrentHeightMode()) {
        // case kAlgae:
        // switch (CoralHeight.getCurrentState()) {
        // case Klevel1:
        // return elevator.p1();
        // break;
        // case Klevel2:
        // return elevator.p2();
        // break;
        // case Klevel3:
        // return elevator.p3();
        // break;
        // case Klevel4:
        // return elevator.p4();
        // break;
        // }
        // break;
        // case kCoral:
        // switch (AlgaeHeight.getCurrentHeightMode()) {
        // case Klevel1:
        // elevator.a1(); // constant unset
        // break;

        // case Klevel2:
        // elevator.a2(); // constant unset
        // break;
        // }
        // default:
        // return null;
        // }
    }
}
