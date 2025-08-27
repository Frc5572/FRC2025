package frc.robot.subsystems.quest;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public interface QuestIO {

    @AutoLog
    public class QuestInputs {
        Pose2d questPose = new Pose2d();
        Rotation2d questYaw = new Rotation2d();
        boolean tracking = false;
        boolean connection = false;
        int batteryPercent = 0;
    }

    public void updateInputs(QuestInputs inputs);

    public void commandPeriodic();

    public void setPose(Pose2d pose);
}
