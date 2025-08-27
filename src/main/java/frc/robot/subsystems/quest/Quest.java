package frc.robot.subsystems.quest;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Quest extends SubsystemBase {
    private QuestIO io;
    private QuestInputsAutoLogged inputs = new QuestInputsAutoLogged();

    public Quest(QuestIO io) {
        this.io = io;
        io.updateInputs(inputs);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }
}
