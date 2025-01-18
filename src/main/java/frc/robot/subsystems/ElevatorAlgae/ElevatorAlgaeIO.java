package frc.robot.subsystems.ElevatorAlgae;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorAlgaeIO {
    @AutoLog
    public class AlgaeIOInputs {
        double AlgaeMotorSpeed;
    }

    public default void setAlgaeMotorSpeed(double speed) {}

    public default void updateInputs(AlgaeIOInputs inputs) {}
}
