package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

    /**
     * converts angles to distance
     *
     * @param angle the angle the motor moves
     * @return distance
     */
    private Distance angleToDistance(Angle angle) {
        return Meters.of(angle.in(Rotation) * Constants.Elevator.gearRatio);
    }

    /**
     * converts distance to angle
     *
     * @param distance the height we want converted
     *
     * @return angle
     * 
     */
    private Angle distanceToAngle(Distance distance) {
        return Rotations.of(distance.in(Meters) / Constants.Elevator.gearRatio);
    }

    /**
     * moves elevator to home
     *
     * @return elevator at home
     * 
     */
    public Command home() {
        return raise(Constants.Elevator.HOME).andThen(() -> io.setVoltage(-0.1))
            .until(() -> (inputs.limitSwitch == true));
    }

    /**
     * moves elevator to l2
     *
     * @return elevator at l2
     * 
     */
    public Command p1() {
        return raise(Constants.Elevator.P1);
    }

    public Command p2() {
        return raise(Constants.Elevator.P2);
    }

    public Command p3() {
        return raise(Constants.Elevator.P3);
    }

    public Command p4() {
        return raise(Constants.Elevator.P4);
    }

    public Command p5() {
        return raise(Constants.Elevator.P5);
    }

    public Command barge() {
        return raise(Constants.Elevator.BARGE);
    }

    // public Command barage() {
    // return raise(Constants.Elevator.L2);
    // }

    /**
     * sets height of elevator
     *
     * @param height desired height of elevator
     * @return elevator height change
     * 
     */
    public Command raise(Distance height) {
        return run(() -> {
            Logger.recordOutput("targetHeight", height.in(Meters));
            io.setPositon(height.in(Meters));
        });
    }

    public Command move() {
        return runEnd(() -> io.setVoltage(SmartDashboard.getNumber("elevatorVoltage", 1.0)),
            () -> io.setVoltage(0));
    }

    public Command moveeng() {
        return runEnd(() -> io.setVoltage(-1.0), () -> io.setVoltage(0));
    }
}
