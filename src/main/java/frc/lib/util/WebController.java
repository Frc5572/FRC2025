package frc.lib.util;

import java.nio.file.Paths;
import java.util.EnumSet;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import io.javalin.Javalin;
import io.javalin.http.staticfiles.Location;

/** Controller via HTTP. */
public class WebController {

    /**
     * Create a new WebController
     * 
     * @param index relative path (from deploy directory) of the folder containing the web contents.
     * @param port Port the webserver is exposed on.
     */
    public WebController(String index, int port) {
        Javalin.create(config -> {
            config.staticFiles
                .add(Paths.get(Filesystem.getDeployDirectory().getAbsolutePath().toString(), index)
                    .toString(), Location.EXTERNAL);
        }).start(port);
        NetworkTableInstance instance = NetworkTableInstance.getDefault();
        IntegerPublisher confirmLoc =
            instance.getIntegerTopic("/coral-selector/coralLocConfirm").publish();
        instance.addListener(instance.getTopic("/coral-selector/coralLoc"),
            EnumSet.of(NetworkTableEvent.Kind.kValueRemote), (ev) -> {
                long level = ev.valueData.value.getInteger();
                confirmLoc.accept(level);
            });
        IntegerPublisher confirmHeight =
            instance.getIntegerTopic("/coral-selector/coralHeightConfirm").publish();
        instance.addListener(instance.getTopic("/coral-selector/coralHeight"),
            EnumSet.of(NetworkTableEvent.Kind.kValueRemote), (ev) -> {
                long level = ev.valueData.value.getInteger();
                confirmHeight.accept(level);
            });
    }

}
