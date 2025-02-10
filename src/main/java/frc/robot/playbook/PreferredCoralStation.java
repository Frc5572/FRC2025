package frc.robot.playbook;

import static edu.wpi.first.units.Units.Meters;
import java.util.function.Function;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.FieldConstants;

public enum PreferredCoralStation {
    Fastest((pose) -> {
        if (pose.getY() > FieldConstants.fieldWidth.in(Meters)) {
            return FieldConstants.CoralStation.leftFeederPose;
        } else {
            return FieldConstants.CoralStation.rightFeederPose;
        }
    }), LeftStation((_pose) -> FieldConstants.CoralStation.leftFeederPose), RightStation(
        (_pose) -> FieldConstants.CoralStation.rightFeederPose);

    public final Function<Pose2d, Pose2d> getPose;

    PreferredCoralStation(Function<Pose2d, Pose2d> getPose) {
        this.getPose = getPose;
    }
}
