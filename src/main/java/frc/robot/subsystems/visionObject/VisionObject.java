package frc.robot.subsystems.visionObject;

import java.util.function.Function;
import java.util.stream.Stream;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.visionObject.VisionObjectIO.VisionObjectInputs;

public class VisionObject extends SubsystemBase {
    private VisionObjectIO io;
    private VisionObjectInputs[] inputs;
    private final RobotState state;
    private Transform3d[] robotToCamera;
    PhotonPipelineResult[] results;
    private String logIntro = "visionObject/";

    public Transform2d targetTransform;

    public VisionObject(RobotState state,
        Function<Constants.Vision.CameraConstants[], VisionObjectIO> io) {
        super("visionObject/");
        this.state = state;
        this.io = io.apply(Constants.Vision.cameras);
        this.robotToCamera = Stream.of(Constants.Vision.cameras).map(x -> x.robotToCamera())
            .toArray(Transform3d[]::new);
        inputs = new VisionObjectInputs[Constants.Vision.cameras.length];
        for (int i = 0; i < Constants.Vision.cameras.length; i++) {
            inputs[i] = new VisionObjectInputs();
        }
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(logIntro, inputs[0]);

        results = inputs[0].results;

        // targetTransform = calcTransform();
        targetTransform = new Transform2d(10.0, 10.0, Rotation2d.k180deg);
        Logger.recordOutput(logIntro + "targetTranslation", targetTransform);
    }



    /** just using slang */

    private double calcDistance() {
        // var result = results[0].getBestTarget();
        return PhotonUtils.calculateDistanceToTargetMeters(robotToCamera[2].getZ(), 1000000,
            robotToCamera[2].getRotation().getAngle(), 1000000);
    }

    private Rotation2d calcRotation() {
        var result = results[0].getBestTarget();
        return new Rotation2d(result.getYaw());
    }

    private Translation2d calcTranslation() {
        var result = results[0].getBestTarget();
        return PhotonUtils.estimateCameraToTargetTranslation(calcDistance(),
            new Rotation2d(result.getYaw()));
    }

    private Transform2d calcTransform() {
        return new Transform2d(calcTranslation(), calcRotation());
    }
}
