package frc.robot.subsystems.swerve;

import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.swerve.SwerveModule;
import frc.lib.util.swerve.SwerveModuleSim;

/** Simulated Swerve Drive */
public class SwerveSim implements SwerveIO {

    private final GyroSimulation gyroSim;
    private final SwerveDriveSimulation simulation;

    public SwerveSim(SwerveDriveSimulation simulation) {
        this.gyroSim = simulation.getGyroSimulation();
        this.simulation = simulation;
    }

    @Override
    public void updateInputs(SwerveInputs inputs) {
        inputs.yawCanAndGyro = gyroSim.getGyroReading().getRotations();
    }

    @Override
    public SwerveModule[] createModules() {
        SwerveModuleSimulation[] simModules = simulation.getModules();
        SwerveModule[] modules = new SwerveModule[simModules.length];
        for (int i = 0; i < modules.length; i++) {
            modules[i] =
                new SwerveModule(i, Rotation2d.kZero, new SwerveModuleSim(i, simModules[i]));
        }

        return modules;
    }

    @Override
    public void setPose(Pose2d pose) {
        simulation.setSimulationWorldPose(pose);
    }

}
