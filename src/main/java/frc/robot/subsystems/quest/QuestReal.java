package frc.robot.subsystems.quest;

import edu.wpi.first.math.geometry.Pose2d;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class QuestReal implements QuestIO {
    QuestNav quest = new QuestNav();
    // Get the latest pose data frames from the Quest
    PoseFrame[] poseFrames = quest.getAllUnreadPoseFrames();

    @Override
    public void updateInputs(QuestInputs inputs) {
        if (poseFrames.length > 0) {
            inputs.questPose = poseFrames[poseFrames.length - 1].questPose().transformBy(null);
            inputs.questYaw =
                poseFrames[poseFrames.length - 1].questPose().transformBy(null).getRotation();
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
        quest.setPose(pose);
    }


}
