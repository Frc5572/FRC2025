package frc.robot.subsystems.swerve;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.util.swerve.SwerveModule;
import frc.lib.util.swerve.SwerveModuleSim;

/**
 * Simulated Swerve Drive
 */
public class SwerveSim implements SwerveIO {

    private final SwerveDriveSimulation simulation;

    /**
     * Simulated Swerve Drive
     */
    public SwerveSim(SwerveDriveSimulation simulation) {

        this.simulation = simulation;
    }

    @Override
    public void updateInputs(SwerveInputs inputs) {}

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

    @Override
    public void setDriveMotorVoltage(Voltage volts) {
        setDriveMotorVoltage(volts);
    }

}
