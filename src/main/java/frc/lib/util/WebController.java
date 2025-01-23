package frc.lib.util;

import java.nio.file.Paths;
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
    }

}
