package frc.robot.subsystems.swerve;

import org.ironmaple.simulation.drivesims.GyroSimulation;
import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.util.swerve.SwerveModule;

/** Simulated Gyro */
public class GyroSim implements SwerveIO {

    private GyroSimulation navXSim;
    private GyroSimulation canandGyroSim;

    public GyroSim(GyroSimulation navXSim, GyroSimulation canandGyroSim) {
        this.navXSim = navXSim;
        this.canandGyroSim = canandGyroSim;
    }

    @Override
    public void updateInputs(SwerveInputs inputs) {
        inputs.yaw = navXSim.getGyroReading().getRotations();
        inputs.yaw = canandGyroSim.getGyroReading().getRotations();
    }

    @Override
    public SwerveModule[] createModules() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'createModules'");
    }

    @Override
    public void setPose(Pose2d pose) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setPose'");
    }

}
