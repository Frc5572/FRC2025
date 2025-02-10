package frc.robot.playbook;

import java.util.ArrayList;
import edu.wpi.first.math.geometry.Pose2d;

public class PlaybookInterface {

    private final ArrayList<Play> plays = new ArrayList<>();
    private PreferredCoralStation preferredCoralStation;
    private PreferredDirection preferredDirection;

    private Pose2d mark;

    private PlaybookInterface() {}

    private static final PlaybookInterface instance = new PlaybookInterface();

    public static PlaybookInterface getInstance() {
        return instance;
    }

}
