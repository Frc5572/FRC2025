package frc.robot.subsystems.quest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

/** quest real class */
public class QuestReal implements QuestIO {
    QuestNav quest = new QuestNav();
    // Get the latest pose data frames from the Quest
    PoseFrame[] poseFrames = quest.getAllUnreadPoseFrames();
    Pose3d pose;


    @Override
    public void updateInputs(QuestInputs inputs) {
        poseFrames = quest.getAllUnreadPoseFrames();

        if (poseFrames.length > 0) {
            pose = poseFrames[poseFrames.length - 1].questPose3d();

            inputs.questPose = pose.toPose2d();
            inputs.questYaw =
                poseFrames[poseFrames.length - 1].questPose3d().toPose2d().getRotation();
        }
        inputs.tracking = quest.isTracking();
        inputs.connection = quest.isConnected();
        inputs.batteryPercent = quest.getBatteryPercent().getAsInt();
    }

    @Override
    public void commandPeriodic() {
        quest.commandPeriodic();
    }

    @Override
    public void setPose(Pose2d pose) {
        quest.setPose(new Pose3d(pose));
    }
}
