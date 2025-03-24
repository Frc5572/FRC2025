package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Rotation;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

/** Simulated Gyro */
public class GyroSim implements GyroIO {

    private GyroSimulation gyro;

    public GyroSim(SwerveDriveSimulation driveSimulation) {
        this.gyro = driveSimulation.getGyroSimulation();
    }

    @Override
    public void updateInputs(GyroInputs inputs) {
        inputs.yaw = Rotation.of(gyro.getGyroReading().getRotations());
    }
}
