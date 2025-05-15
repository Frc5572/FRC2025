package frc.robot.subsystems.visionObject;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Milliseconds;
import java.util.function.Function;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import frc.robot.Constants;

public class VisionObjectSim extends VisionObjectReal {
    private final SwerveDriveSimulation sim;
    private final VisionSystemSim visionSim;

    public static Function<Constants.Vision.CameraConstants[], VisionObjectIO> partial(
        SwerveDriveSimulation sim) {
        return (constants) -> new VisionObjectSim(constants, sim);
    }

    public VisionObjectSim(Constants.Vision.CameraConstants[] constants,
        SwerveDriveSimulation sim) {
        super(constants);
        this.sim = sim;
        visionSim = new VisionSystemSim("main");

        visionSim.addAprilTags(Constants.Vision.fieldLayout);

        // visionSim.addVisionTargets(VisionTargetSim);

        for (int i = 0; i < constants.length; i++) {
            SimCameraProperties props = new SimCameraProperties();
            props.setCalibration(constants[i].width(), constants[i].height(),
                constants[i].horizontalFieldOfView());
            props.setCalibError(constants[i].calibErrorAvg(), constants[i].calibErrorStdDev());
            props.setFPS(constants[i].framesPerSecond().in(Hertz));
            props.setAvgLatencyMs(constants[i].latencyAvg().in(Milliseconds));
            props.setLatencyStdDevMs(constants[i].latencyStdDev().in(Milliseconds));
            PhotonCameraSim cameraSim = new PhotonCameraSim(this.camera[i], props);
            visionSim.addCamera(cameraSim, constants[i].robotToCamera());
        }
    }

    @Override
    public void updateInputs(VisionObjectInputs[] inputs) {
        visionSim.update(sim.getSimulatedDriveTrainPose());
        super.updateInputs(inputs);
    }

    @Override
    protected void createSettingsUploadThread(String hostname) {
        System.out.println("Not uploading settings for PV Sim: " + hostname);
    }
}
