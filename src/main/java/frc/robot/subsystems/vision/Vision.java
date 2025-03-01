package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;
import java.util.stream.IntStream;
import java.util.stream.Stream;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.IntArrayList;
import frc.lib.util.LoggedTracer;
import frc.lib.util.Tuples.Tuple3;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.vision.VisionIO.CameraInputs;

/** Vision Subsystem */
public class Vision extends SubsystemBase {

    private final VisionIO io;
    private final CameraInputs[] cameraInputs;

    private final RobotState state;

    private final Transform3d[] robotToCamera;

    private boolean seesMultitag = false;
    public Trigger seesTwoAprilTags = new Trigger(() -> twoAprilTags());

    /** Vision Subsystem */
    public Vision(RobotState state, Function<Constants.Vision.CameraConstants[], VisionIO> io) {
        super("Vision");
        this.state = state;
        this.io = io.apply(Constants.Vision.cameras);
        this.robotToCamera = Stream.of(Constants.Vision.cameras).map(x -> x.robotToCamera())
            .toArray(Transform3d[]::new);
        cameraInputs = new CameraInputs[Constants.Vision.cameras.length];
        for (int i = 0; i < Constants.Vision.cameras.length; i++) {
            cameraInputs[i] = new CameraInputs();
        }
    }

    private final IntArrayList tmpArrList = new IntArrayList();

    @Override
    public void periodic() {
        io.updateInputs(cameraInputs);
        for (int i = 0; i < cameraInputs.length; i++) {
            Logger.processInputs("Camera" + i, cameraInputs[i]);
        }

        // Update RobotState
        List<Tuple3<Integer, Transform3d, PhotonPipelineResult>> results = new ArrayList<>();
        for (int i = 0; i < cameraInputs.length; i++) {
            var transform = robotToCamera[i];

            for (int j = 0; j < cameraInputs[i].results.length; j++) {
                var result = cameraInputs[i].results[j];
                results.add(new Tuple3<>(i, transform, result));
            }
        }

        results.sort(
            (a, b) -> Double.compare(a._2().getTimestampSeconds(), b._2().getTimestampSeconds()));
        for (var result : results) {
            if (result._2().multitagResult.isPresent()) {
                seesMultitag = true;
            } else if (result._0() == 0) {
                seesMultitag = false;
            }
            state.addVisionObservation(result._2(), result._1(), result._0());
        }
        // Viz
        Pose3d robotPose = new Pose3d(state.getGlobalPoseEstimate());
        for (int i = 0; i < cameraInputs.length; i++) {
            IntArrayList cameraTags = tmpArrList;
            for (int j = 0; j < cameraInputs[i].results.length; j++) {
                var result = cameraInputs[i].results[j];
                for (var target : result.targets) {
                    cameraTags.add(target.fiducialId);
                }
            }
            Pose3d cameraPose = robotPose.plus(robotToCamera[i]);
            Pose3d[] draw = IntStream.of(cameraTags.toArray()).distinct()
                .mapToObj(id -> Constants.Vision.fieldLayout.getTagPose(id))
                .filter(Optional::isPresent).flatMap(x -> Stream.of(x.get(), cameraPose))
                .toArray(Pose3d[]::new);
            cameraTags.clear();
            Logger.recordOutput("Vision/Camera" + i + "/AprilTags", draw);
            if (draw.length != 0) {
                Logger.recordOutput("Vision/Camera" + i + "/AprilTagsCached", draw);
            }
        }
        LoggedTracer.record("Vision");


    }

    public boolean twoAprilTags() {
        return seesMultitag && edu.wpi.first.wpilibj.RobotState.isDisabled();
    }

}
