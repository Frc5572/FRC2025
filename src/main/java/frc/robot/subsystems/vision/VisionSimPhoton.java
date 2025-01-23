package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Milliseconds;
import java.util.function.Function;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import frc.robot.Constants;

/** Simulation of vision using built-in PhotonVision simulator. */
public class VisionSimPhoton extends VisionReal {

    private final SwerveDriveSimulation sim;
    private final VisionSystemSim visionSim;

    /** Simulation of vision using built-in PhotonVision simulator. */
    public static Function<Constants.Vision.CameraConstants[], VisionIO> partial(
        SwerveDriveSimulation sim) {
        return (constants) -> new VisionSimPhoton(constants, sim);
    }

    /** Simulation of vision using built-in PhotonVision simulator. */
    public VisionSimPhoton(Constants.Vision.CameraConstants[] constants,
        SwerveDriveSimulation sim) {
        super(constants);
        this.sim = sim;
        visionSim = new VisionSystemSim("main");

        AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        visionSim.addAprilTags(tagLayout);

        for (int i = 0; i < constants.length; i++) {
            SimCameraProperties props = new SimCameraProperties();
            props.setCalibration(constants[i].width(), constants[i].height(),
                constants[i].horizontalFieldOfView());
            props.setCalibError(constants[i].calibErrorAvg(), constants[i].calibErrorStdDev());
            props.setFPS(constants[i].framesPerSecond().in(Hertz));
            props.setAvgLatencyMs(constants[i].latencyAvg().in(Milliseconds));
            props.setLatencyStdDevMs(constants[i].latencyStdDev().in(Milliseconds));
            PhotonCameraSim cameraSim = new PhotonCameraSim(this.cameras[i], props);
            visionSim.addCamera(cameraSim, constants[i].robotToCamera());
        }
    }

    @Override
    public void updateInputs(CameraInputs[] inputs) {
        visionSim.update(sim.getSimulatedDriveTrainPose());
        super.updateInputs(inputs);
    }

}
