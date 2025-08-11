package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.ScoringLocation;
import frc.lib.util.ScoringLocation.CoralLocation;
import frc.lib.util.ScoringLocation.Height;
import frc.robot.CommandFactory;
import frc.robot.subsystems.algaewrist.AlgaeWrist;
import frc.robot.subsystems.coral.CoralScoring;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator_algae.ElevatorAlgae;
import frc.robot.subsystems.swerve.Swerve;

public class AutoScore {

    public static class PoseWithFlag {
        private final Supplier<ScoringLocation.CoralLocation> location;
        private final Supplier<ScoringLocation.Height> height;
        private boolean flag;

        public PoseWithFlag(Supplier<ScoringLocation.CoralLocation> location,
            Supplier<ScoringLocation.Height> height, boolean flag) {
            this.location = location;
            this.height = height;
            this.flag = flag;
        }

        public Supplier<ScoringLocation.CoralLocation> getLocation() {
            return location;
        }

        public Supplier<ScoringLocation.Height> getHeight() {
            return height;
        }

        public Pose2d getCoralPose() {
            return location.get().pose;
        }

        public boolean isFlag() {
            return flag;
        }

        public void setFlag(boolean flag) {
            this.flag = flag;
        }
    }

    private final PoseWithFlag[] poseWithFlagsL4;
    private final PoseWithFlag[] poseWithFlagsL3;
    private final PoseWithFlag[] poseWithFlagsL1;

    public AutoScore() {
        CoralLocation[] locations = CoralLocation.values();
        poseWithFlagsL4 = new PoseWithFlag[locations.length];
        poseWithFlagsL3 = new PoseWithFlag[locations.length];
        poseWithFlagsL1 = new PoseWithFlag[locations.length];

        for (int i = 0; i < locations.length; i++) {
            final CoralLocation loc = locations[i];
            poseWithFlagsL4[i] = new PoseWithFlag(() -> loc, () -> Height.KP4, true);
            poseWithFlagsL3[i] = new PoseWithFlag(() -> loc, () -> Height.KP3, true);
            poseWithFlagsL1[i] = new PoseWithFlag(() -> loc, () -> Height.KP1, true);
        }
    }

    private Command autoScoreCMD(Swerve swerve, Elevator elevator, CoralScoring coralScoring,
        ElevatorAlgae algae, AlgaeWrist wrist, PoseWithFlag[] poses) {
        List<Command> sequenceCommands = new ArrayList<>();

        for (PoseWithFlag p : poses) {
            if (p.isFlag()) {
                p.setFlag(false);
                Command scoreCmd = CommandFactory.maybeScoreCoral(swerve, elevator, coralScoring,
                    algae, wrist, p.getLocation(), p.getHeight());
                Command feederCmd = CommandFactory.fasterFeeder(swerve, elevator, coralScoring);

                sequenceCommands.add(scoreCmd.andThen(feederCmd));
            }
        }

        if (sequenceCommands.isEmpty()) {
            return Commands.runOnce(() -> Logger.recordOutput("AutoScore", "Nothing to score"));
        }

        return Commands.sequence(sequenceCommands.toArray(new Command[0]));
    }

    public Command TopAll(Swerve swerve, Elevator elevator, CoralScoring coralScoring,
        ElevatorAlgae algae, AlgaeWrist wrist) {
        return autoScoreCMD(swerve, elevator, coralScoring, algae, wrist, poseWithFlagsL4);
    }

    public Command midAll(Swerve swerve, Elevator elevator, CoralScoring coralScoring,
        ElevatorAlgae algae, AlgaeWrist wrist) {
        return autoScoreCMD(swerve, elevator, coralScoring, algae, wrist, poseWithFlagsL3);
    }

    public Command lowAll(Swerve swerve, Elevator elevator, CoralScoring coralScoring,
        ElevatorAlgae algae, AlgaeWrist wrist) {
        return autoScoreCMD(swerve, elevator, coralScoring, algae, wrist, poseWithFlagsL1);
    }

    public void resetAllFlags() {
        resetFlags(poseWithFlagsL4);
        resetFlags(poseWithFlagsL3);
        resetFlags(poseWithFlagsL1);
    }

    private void resetFlags(PoseWithFlag[] arr) {
        for (PoseWithFlag p : arr)
            p.setFlag(true);
    }



    public void elasticButtons() {
        SmartDashboard.putBoolean("jL1", true);
        poseWithFlagsL1[9].setFlag(SmartDashboard.getBoolean("jL1", true));
        SmartDashboard.putBoolean("jL3", true);
        poseWithFlagsL3[9].setFlag(SmartDashboard.getBoolean("jL3", true));
        SmartDashboard.putBoolean("jL4", true);
        poseWithFlagsL4[9].setFlag(SmartDashboard.getBoolean("jL4", true));
    }
}
