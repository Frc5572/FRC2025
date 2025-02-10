package frc.robot.playbook;

import java.nio.file.Paths;
import java.util.EnumSet;
import java.util.concurrent.ArrayBlockingQueue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.ScoringLocation.AlgaeHeight;
import frc.lib.util.ScoringLocation.AlgaeLocation;
import frc.lib.util.ScoringLocation.CoralHeight;
import frc.lib.util.ScoringLocation.CoralLocation;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.swerve.Swerve;
import io.javalin.Javalin;
import io.javalin.http.staticfiles.Location;

public class Playbook extends Command {

    private final ArrayBlockingQueue<Play> plays = new ArrayBlockingQueue<>(20);
    private PreferredCoralStation preferredCoralStation;
    private PreferredDirection preferredDirection;

    private Pose2d mark;

    private final PlayCommandArgs args;

    private Command currentCommand = null;

    private final StringPublisher confirm;

    public Playbook(Swerve swerve, Elevator elevator, Trigger hasCoral, Trigger hasAlgae) {
        Javalin.create(config -> {
            config.staticFiles
                .add(Paths.get(Filesystem.getDeployDirectory().getAbsolutePath().toString(), "web")
                    .toString(), Location.EXTERNAL);
        }).start(5800);
        NetworkTableInstance instance = NetworkTableInstance.getDefault();
        confirm = instance.getStringTopic("/playbook/confirm").publish();
        instance.addListener(instance.getTopic("/playbook/command"),
            EnumSet.of(NetworkTableEvent.Kind.kValueRemote), (ev) -> {
                String command = ev.valueData.value.getString();
                if (command.isEmpty()) {
                    return;
                }
                addPlay(command);
                confirm.set(getCommandString());
            });
        args = new PlayCommandArgs(swerve, elevator, newMark -> {
            this.mark = newMark;
        }, () -> this.mark, () -> this.preferredCoralStation, () -> this.preferredDirection,
            hasCoral, hasAlgae);
    }

    public void addPlay(String s) {
        System.out.println("adding command: " + s);
        if (s.charAt(0) == 'a') {
            CoralLocation location = CoralLocation.fromChar(s.charAt(1));
            AlgaeHeight height = AlgaeHeight.fromChar(s.charAt(2));
            AlgaeLocation targetLocation = AlgaeLocation.fromChar(s.charAt(3));
            if (location != null && height != null && targetLocation != null) {
                plays.add(new AlgaePlay(location, height, targetLocation));
                return;
            }
        } else if (s.charAt(0) == 'c') {
            CoralLocation location = CoralLocation.fromChar(s.charAt(1));
            CoralHeight height = CoralHeight.fromChar(s.charAt(2));
            if (location != null && height != null) {
                plays.add(new CoralPlay(location, height));
                return;
            }
        } else if (s.charAt(0) == 'd') {
            if (s.charAt(1) == 'm') {
                plays.add(new MarkPlay());
                return;
            } else if (s.charAt(1) == 'r') {
                plays.add(new RestorePlay());
                return;
            } else if (s.charAt(1) == 'c') {
                plays.clear();
                end(true);
                return;
            }
        } else if (s.charAt(0) == 'f') {
            if (s.charAt(1) == 'l') {
                preferredCoralStation = PreferredCoralStation.LeftStation;
                return;
            } else if (s.charAt(1) == 'r') {
                preferredCoralStation = PreferredCoralStation.RightStation;
                return;
            } else if (s.charAt(1) == 'f') {
                preferredCoralStation = PreferredCoralStation.Fastest;
                return;
            }
        }
        System.err.println("Invalid play string: " + s);
    }

    private String getCommandString() {
        StringBuilder builder = new StringBuilder();
        for (Play p : plays) {
            builder.append(p.getCommandString());
        }
        return builder.toString();
    }

    @Override
    public void initialize() {
        System.out.println("initialize");
        if (currentCommand == null) {
            System.out.println("getting next play");
            var next = plays.poll();
            if (next != null) {
                System.out.println("next play is not null");
                currentCommand = next.asCommand(args);
                currentCommand.initialize();
                confirm.set(getCommandString());
            }
        }
    }

    @Override
    public void execute() {
        initialize();
        if (currentCommand != null) {
            currentCommand.execute();
            if (currentCommand.isFinished()) {
                currentCommand.end(false);
                currentCommand = null;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        if (currentCommand != null) {
            currentCommand.end(interrupted);
        }
    }

}
