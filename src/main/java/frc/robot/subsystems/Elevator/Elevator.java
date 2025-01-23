package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Rotation;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Elevator Subsystem
 */
public class Elevator extends SubsystemBase {
    ElevatorIO io;
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

    // private Distance angleToDistnce(Angle angle) {
    // x = Meter.of(angle);
    // return Meter.of(9);
    // }

    public Command home() {
        return raise(Constants.Elevator.HOME);
    }

    public Command l2() {
        return raise(Constants.Elevator.L2);
    }

    public Command l3() {
        return raise(Constants.Elevator.L3);
    }

    public Command l4() {
        return raise(Constants.Elevator.L4);
    }

    public Command raise(double positon) {
        return runOnce(() -> io.setPositon(positon))
            .andThen(Commands.waitUntil(() -> inputs.positon.in(Rotation) == positon));
    }
}
