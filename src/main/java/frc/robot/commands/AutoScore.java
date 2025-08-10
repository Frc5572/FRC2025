package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
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
    public class PoseWithFlag {
        private Supplier<ScoringLocation.CoralLocation> location;
        private Supplier<ScoringLocation.Height> height;
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

        public Pose2d getCoralPose(Supplier<ScoringLocation.CoralLocation> location) {
            return location.get().pose;
        }



        public boolean isFlag() {
            return flag;
        }

        public void setFlag(boolean flag) {
            this.flag = flag;
        }
    }

    private PoseWithFlag[] poseWithFlags;

    public AutoScore() {
        CoralLocation[] locations = CoralLocation.values();
        poseWithFlags = new PoseWithFlag[locations.length];
        for (int i = 0; i < locations.length; i++) {
            int index = i;
            poseWithFlags[i] = new PoseWithFlag(() -> locations[index], null, true);
        }
    }

    public Command autoScoreCMD(Swerve swerve, Elevator elevator, CoralScoring coralScoring,
        ElevatorAlgae algae, AlgaeWrist wrist, Height targetHeight) {

        var commandGroup = Commands.sequence();

        for (PoseWithFlag pose : poseWithFlags) {
            if (pose.isFlag()) {
                pose.setFlag(false);
                Supplier<Height> heightSupplier = () -> targetHeight;

                Command scoreCmd = CommandFactory.maybeScoreCoral(swerve, elevator, coralScoring,
                    algae, wrist, pose.getLocation(), heightSupplier);
                Command feederCmd = CommandFactory.fasterFeeder(swerve, elevator, coralScoring);

                commandGroup = Commands.sequence(commandGroup, scoreCmd.andThen(feederCmd));
            }
        }

        return commandGroup;
    }



}
