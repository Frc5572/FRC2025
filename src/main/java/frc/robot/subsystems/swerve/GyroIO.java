package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Rotation;
import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.Angle;

/** Gyro IO layer */
public interface GyroIO {
    /** Gyro input logger */
    @AutoLog
    public class GyroInputs {
        Angle yaw = Rotation.of(0);
    }

    public void updateInputs(GyroInputs inputs);

    /** Empty Swerve implementation (for replay) */
    public static class Empty implements GyroIO {
        @Override
        public void updateInputs(GyroInputs inputs) {}
    }
}
