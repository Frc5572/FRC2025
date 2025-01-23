package frc.robot.subsystems.vision;

import java.util.function.Function;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.vision.VisionIO.CameraInputs;

/** Vision Subsystem */
public class Vision extends SubsystemBase {

    private final VisionIO io;
    private final CameraInputs[] cameraInputs;

    /** Vision Subsystem */
    public Vision(Function<Constants.Vision.CameraConstants[], VisionIO> io) {
        super("Vision");
        this.io = io.apply(Constants.Vision.cameras);
        cameraInputs = new CameraInputs[Constants.Vision.cameras.length];
        for (int i = 0; i < Constants.Vision.cameras.length; i++) {
            cameraInputs[i] = new CameraInputs();
        }
    }

    @Override
    public void periodic() {
        io.updateInputs(cameraInputs);
        for (int i = 0; i < cameraInputs.length; i++) {
            Logger.processInputs("Camera" + i, cameraInputs[i]);
        }
    }

}
