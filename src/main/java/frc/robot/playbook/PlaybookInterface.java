package frc.robot.playbook;

import java.nio.file.Paths;
import java.util.ArrayList;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import io.javalin.Javalin;
import io.javalin.http.staticfiles.Location;

public class PlaybookInterface {

    private final ArrayList<Play> plays = new ArrayList<>();
    private PreferredCoralStation preferredCoralStation;
    private PreferredDirection preferredDirection;

    private Pose2d mark;

    private PlaybookInterface() {
        Javalin.create(config -> {
            config.staticFiles
                .add(Paths.get(Filesystem.getDeployDirectory().getAbsolutePath().toString(), "web")
                    .toString(), Location.EXTERNAL);
        }).start(5800);
        NetworkTableInstance instance = NetworkTableInstance.getDefault();
    }

    private static final PlaybookInterface instance = new PlaybookInterface();

    public static PlaybookInterface getInstance() {
        return instance;
    }

}
