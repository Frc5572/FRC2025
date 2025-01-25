package frc.robot.subsystems.vision;

import java.util.stream.Stream;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import frc.robot.Constants;

/** PhotonVision-attached implementation */
public class VisionReal implements VisionIO {

    protected final PhotonCamera[] cameras;

    /** PhotonVision-attached implementation */
    public VisionReal(Constants.Vision.CameraConstants[] constants) {
        cameras = Stream.of(constants).map((consts) -> new PhotonCamera(consts.name()))
            .toArray(PhotonCamera[]::new);
    }

    @Override
    public void updateInputs(CameraInputs[] inputs) {
        for (int i = 0; i < cameras.length; i++) {
            inputs[i].results =
                cameras[i].getAllUnreadResults().toArray(PhotonPipelineResult[]::new);
            inputs[i].cameraMatrix = cameras[i].getCameraMatrix();
            inputs[i].distCoeffs = cameras[i].getDistCoeffs();
        }
    }

}
