package frc.robot.playbook;

import edu.wpi.first.wpilibj2.command.Command;

public sealed interface Play permits AlgaePlay, CoralPlay, MarkPlay, RestorePlay {

    public String getCommandString();

    public Command asCommand(PlayCommandArgs args);

}
