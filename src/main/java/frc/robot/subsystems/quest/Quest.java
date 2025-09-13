package frc.robot.subsystems.quest;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;

public class Quest extends SubsystemBase {
    private QuestIO io;
    private QuestInputsAutoLogged inputs = new QuestInputsAutoLogged();
    private Pose2d posInit;
    private RobotState state;

    public Quest(QuestIO io, RobotState state) {
        this.io = io;
        this.state = state;
        io.updateInputs(inputs);
        posInit = state.getGlobalPoseEstimate();
        io.setPose(posInit);
        Logger.recordOutput("Quest/PosInit", posInit);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Quest", inputs);
        if (posInit == null) {
            posInit = state.getGlobalPoseEstimate();
            io.setPose(posInit);
        }
        Logger.recordOutput("Quest/PosInit", posInit);
        Logger.recordOutput("Quest/getPose", getPose());
    }

    public Pose2d getPose() {
        return inputs.questPose;
    }

    public Command setPos() {
        return Commands.runOnce(() -> io.setPose(state.getGlobalPoseEstimate()));
    }
}
