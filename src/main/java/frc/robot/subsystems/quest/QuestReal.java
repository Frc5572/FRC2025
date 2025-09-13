package frc.robot.subsystems.quest;

import static edu.wpi.first.units.Units.Inches;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class QuestReal implements QuestIO {
    QuestNav quest = new QuestNav();
    // Get the latest pose data frames from the Quest
    PoseFrame[] poseFrames = quest.getAllUnreadPoseFrames();
    Pose2d pose;
    Transform2d robotToQuest = new Transform2d(new Translation2d(Inches.of(-11.2), Inches.of(9.75)),
        Rotation2d.kCCW_90deg);

    @Override
    public void updateInputs(QuestInputs inputs) {
        poseFrames = quest.getAllUnreadPoseFrames();

        if (poseFrames.length > 0) {
            pose = poseFrames[poseFrames.length - 1].questPose();

            inputs.questPose = pose.transformBy(robotToQuest.inverse());
            inputs.questYaw = poseFrames[poseFrames.length - 1].questPose().getRotation();
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
