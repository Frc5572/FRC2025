package frc.robot.playbook;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class MarkPlay implements Play {

    @Override
    public String getCommandString() {
        return "dm";
    }

    @Override
    public Command asCommand(PlayCommandArgs args) {
        return Commands.runOnce(() -> {
            args.setMark().accept(args.swerve().getPose());
        });
    }

}
