package frc.robot.subsystems.visionObject;

import java.util.function.Function;
import java.util.stream.Stream;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.visionObject.VisionObjectIO.VisionObjectInputs;

public class VisionObject extends SubsystemBase {
    private VisionObjectIO io;
    private VisionObjectInputs inputs;
    private Transform3d[] robotToCamera;
    PhotonPipelineResult[] results;
    private String logIntro = "visionObject/";

    public Transform2d targetTransform = new Transform2d(10.0, 10.0, Rotation2d.k180deg);

    public VisionObject(Function<Constants.Vision.CameraConstants[], VisionObjectIO> io) {
        super("visionObject/");
        this.io = io.apply(Constants.Vision.cameras);
        this.robotToCamera = Stream.of(Constants.Vision.cameras).map(x -> x.robotToCamera())
            .toArray(Transform3d[]::new);
        inputs = new VisionObjectInputs();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(logIntro, inputs);

        results = inputs.results;

        // targetTransform = calcTransform();
        Logger.recordOutput(logIntro + "targetTranslation", targetTransform);
    }

    public Transform2d targetTransform() {
        return targetTransform;
    }

    private Transform2d calcTransform() {
        var distance = PhotonUtils.calculateDistanceToTargetMeters(robotToCamera[2].getZ(),
            results[0].getBestTarget().area, robotToCamera[2].getRotation().getAngle(),
            results[0].getBestTarget().getYaw());
        var yaw = new Rotation2d(results[0].getBestTarget().getYaw());
        var translation = PhotonUtils.estimateCameraToTargetTranslation(distance, yaw);
        return new Transform2d(translation, yaw);
    }
}
