package frc.robot.subsystems.swerve;

import org.ironmaple.simulation.drivesims.GyroSimulation;

/** Simulated Gyro */
public class GyroSim implements GyroIO {

    private GyroSimulation gyro;

    public GyroSim(GyroSimulation gyro) {
        this.gyro = gyro;
    }

    @Override
    public void updateInputs(GyroInputs inputs) {
        inputs.yaw = gyro.getGyroReading().getRotations();
    }
}
