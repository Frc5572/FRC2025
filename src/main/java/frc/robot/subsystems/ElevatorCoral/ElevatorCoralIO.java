package frc.robot.subsystems.ElevatorCoral;

import org.littletonrobotics.junction.AutoLog;
public interface ElevatorCoralIO {
    @AutoLog
    public class ElevatorCoralIOInputs {
        double feederMotorSpeed;

    }

    public default void updateInputs(ElevatorCoralIOInputs inputs) {}

    public default void setFeederMotorSpeed(double speed) {}

}
