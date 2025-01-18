package frc.robot.subsystems.ElevatorCoral;

import org.littletonrobotics.junction.AutoLog;
import frc.robot.subsystems.ElevatorAlgae.ElevatorAlgaeIO.AlgaeIOInputs;

public interface ElevatorCoralIO {
    @AutoLog
    public class ElevatorAlgaeIOInputs {
        double feederMotorSpeed;

    }

    public default void updateInputs(AlgaeIOInputs inputs) {}

    public default void setFeederMotorSpeed(double speed) {}

}
