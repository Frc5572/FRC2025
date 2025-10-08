package frc.robot.subsystems.quest;

import static edu.wpi.first.units.Units.Inches;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;

/** quest subsystem */
public class Quest extends SubsystemBase {
    private QuestIO io;
    private QuestInputsAutoLogged inputs = new QuestInputsAutoLogged();
    private Pose2d posInit;
    Transform2d robotToQuest = new Transform2d(new Translation2d(Inches.of(-11.2), Inches.of(9.75)),
        Rotation2d.fromDegrees(-90)); // TRANSFORM INCORRECT NEEDS FIXING
    private RobotState state;
    private boolean hasSet = false;

    /** Quest Constructer and initlizer */
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
        Logger.recordOutput("Quest/PosInit", posInit);
        Logger.recordOutput("Quest/ProcessedPose", getPose());

        if (state.isInitialized() && (hasSet == false)) {
            setPose();
            hasSet = true;
        }
    }

    /**
     * Method for retreaving curret processed questPose
     *
     * @return Current processed questPose
     */

    public Pose2d getPose() {
        return inputs.questPose.transformBy(robotToQuest);
    }

    /**
     * Sets the pose to the current location in robotStates global estmate
     *
     * @return sets quest pose
     */

    public Command setPose() {
        return Commands.runOnce(() -> io.setPose(state.getGlobalPoseEstimate()));
    }
}
