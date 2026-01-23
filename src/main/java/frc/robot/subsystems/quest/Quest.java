package frc.robot.subsystems.quest;

import static edu.wpi.first.units.Units.Inches;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;

/** quest subsystem */
public class Quest extends SubsystemBase {
    private QuestIO io;
    private QuestInputsAutoLogged inputs = new QuestInputsAutoLogged();
    private Pose3d posInit;
    Transform3d robotToQuest =
        new Transform3d(new Translation3d(Inches.of(-11.2), Inches.of(9.75), Inches.zero()),
            new Rotation3d(Rotation2d.fromDegrees(-90))); // TRANSFORM INCORRECT NEEDS FIXING
    private RobotState state;
    private boolean hasSet = false;

    /** Quest Constructor and initlizer */
    public Quest(QuestIO io, RobotState state) {
        this.io = io;
        this.state = state;
        io.updateInputs(inputs);
        posInit = new Pose3d(state.getGlobalPoseEstimate());
        io.setPose(posInit);
        Logger.recordOutput("Quest/PosInit", posInit);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Quest", inputs);
        Logger.recordOutput("Quest/PosInit", posInit);
        Logger.recordOutput("Quest/ProcessedPose", getPose());
    }

    /**
     * Method for retreaving current processed questPose
     *
     * @return Current processed questPose
     */

    public Pose3d getPose() {
        return inputs.questPose.transformBy(robotToQuest);
    }

    /**
     * Sets the pose to the current location in robotStates global estmate
     *
     * @return sets quest pose
     */

    public Command setPose() {
        return Commands.either(Commands.runOnce(() -> {
            io.setPose(new Pose3d(state.getGlobalPoseEstimate()));
            hasSet = true;
        }), Commands.none(), () -> !hasSet).ignoringDisable(true);
    }
}
