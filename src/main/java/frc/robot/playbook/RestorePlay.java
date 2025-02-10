package frc.robot.playbook;

import edu.wpi.first.wpilibj2.command.Command;

public final class RestorePlay implements Play {

    @Override
    public String getCommandString() {
        return "dr";
    }

    @Override
    public Command asCommand(PlayCommandArgs args) {
        return args.swerve().moveAndAvoidReef(args.getMark(), false, 0.1, 1);
    }

}
