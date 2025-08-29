package frc.robot.subsystems.quest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.viz.Viz2025;
import frc.robot.RobotState;

public class Quest extends SubsystemBase {
    private QuestIO io;
    private QuestInputsAutoLogged inputs = new QuestInputsAutoLogged();
    private Viz2025 viz;
    private RobotState state;
    private Pose2d posInit;

    public Quest(QuestIO io, Viz2025 viz) {
        this.io = io;
        this.viz = viz;
        state = new frc.robot.RobotState(viz);
        io.updateInputs(inputs);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        if (posInit == null) {
            posInit = state.getGlobalPoseEstimate();
            io.setPose(posInit);
        }
    }
}
