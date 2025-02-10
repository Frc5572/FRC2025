package frc.robot.playbook;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.ScoringLocation;
import frc.lib.util.ScoringLocation.AlgaeHeight;
import frc.lib.util.ScoringLocation.AlgaeLocation;
import frc.lib.util.ScoringLocation.CoralLocation;

public final class AlgaePlay implements Play {

    public final ScoringLocation.CoralLocation location;
    public final ScoringLocation.AlgaeHeight height;
    public final ScoringLocation.AlgaeLocation targetLocation;

    public AlgaePlay(CoralLocation location, AlgaeHeight height, AlgaeLocation targetLocation) {
        this.location = location;
        this.height = height;
        this.targetLocation = targetLocation;
    }

    @Override
    public String getCommandString() {
        return "a" + location.toString() + height.toString();
    }

    @Override
    public Command asCommand(PlayCommandArgs args) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'asCommand'");
    }

}
