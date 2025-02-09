package frc.lib.util;

import java.nio.file.Paths;
import java.util.EnumSet;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import frc.lib.util.ScoringLocation.CoralHeight;
import frc.lib.util.ScoringLocation.CoralLocation;
import io.javalin.Javalin;
import io.javalin.http.staticfiles.Location;

/** Controller via HTTP. */
public class WebController {

    private ScoringLocation.CoralHeight currentHeight = ScoringLocation.CoralHeight.Klevel1;
    private ScoringLocation.CoralLocation currentLocation = ScoringLocation.CoralLocation.A;

    /**
     * Create a new WebController
     *
     * @param port Port the webserver is exposed on.
     */
    public WebController(int port) {
        Javalin.create(config -> {
            config.staticFiles
                .add(Paths.get(Filesystem.getDeployDirectory().getAbsolutePath().toString(), "web")
                    .toString(), Location.EXTERNAL);
        }).start(port);
        NetworkTableInstance instance = NetworkTableInstance.getDefault();
        IntegerPublisher confirmLoc =
            instance.getIntegerTopic("/coral-selector/coralLocConfirm").publish();
        instance.addListener(instance.getTopic("/coral-selector/coralLoc"),
            EnumSet.of(NetworkTableEvent.Kind.kValueRemote), (ev) -> {
                long level = ev.valueData.value.getInteger();
                currentLocation = CoralLocation.fromInt((int) level);
                confirmLoc.accept(level);
            });
        IntegerPublisher confirmHeight =
            instance.getIntegerTopic("/coral-selector/coralHeightConfirm").publish();
        instance.addListener(instance.getTopic("/coral-selector/coralHeight"),
            EnumSet.of(NetworkTableEvent.Kind.kValueRemote), (ev) -> {
                long level = ev.valueData.value.getInteger();
                currentHeight = CoralHeight.fromInt((int) level);
                confirmHeight.accept(level);
            });
    }

    /** Get the current location selected by the webcontroller */
    public ScoringLocation.CoralLocation getDesiredLocation() {
        return currentLocation;
    }

    /** Get the current height selected by the webcontroller */
    public ScoringLocation.CoralHeight getDesiredHeight() {
        return currentHeight;
    }

}
