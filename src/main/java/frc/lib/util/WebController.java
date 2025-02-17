package frc.lib.util;

import java.nio.file.Paths;
import java.util.EnumSet;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.javalin.Javalin;
import io.javalin.http.staticfiles.Location;

/** Controller via HTTP. */
public class WebController {

    private ScoringLocation.ElevatorHeight currentHeight = null;
    private ScoringLocation.CoralLocation currentLocation = null;
    private char bay = ' ';
    private boolean right = false;
    private int height = -1;
    private char fdr = 'f';

    private boolean[] bays_compl = new boolean[6];
    private boolean[][] branches_compl = new boolean[6][10];

    private final Trigger hasReefLocation =
        new Trigger(() -> currentHeight != null && currentLocation != null);

    private final IntegerArrayPublisher response;

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
        response = instance.getIntegerArrayTopic("/web/resp").publish();
        instance.addListener(instance.getTopic("/web/ctrl"),
            EnumSet.of(NetworkTableEvent.Kind.kValueRemote), (ev) -> {
                String value = ev.valueData.value.getString();
                if (value.startsWith("reef-")) {
                    value = value.substring(5);
                    bay = value.charAt(0);
                    checkReefLocation();
                    createResponse();
                } else if (value.startsWith("p")) {
                    value = value.substring(1);
                    height = Integer.parseInt(value.substring(0, 1));
                    value = value.substring(1);
                    if (value.length() == 0 || value.charAt(0) == 'r') {
                        right = true;
                    } else {
                        right = false;
                    }
                    checkReefLocation();
                    createResponse();
                } else if (value.startsWith("fdr")) {
                    value = value.substring(3);
                    fdr = value.charAt(0);
                    createResponse();
                } else if (value.startsWith("conf")) {
                    value = value.substring(4);
                    switch (value.charAt(0)) {
                        case 'x':
                            if (hasReefLocation.getAsBoolean()) {
                                branches_compl[bay - 'a'][encode_height(height, right)] = true;
                                checkBayCompl();
                                createResponse();
                            }
                            break;
                        case 'y':
                            if (hasReefLocation.getAsBoolean()) {
                                branches_compl[bay - 'a'][encode_height(height, right)] = false;
                                checkBayCompl();
                                createResponse();
                            }
                            break;
                        case 'c':
                            // IDK what this is for
                            break;
                        default:
                            break;
                    }
                }
            });
        createResponse();
    }

    private final long[] response_values = new long[21];
    private static final boolean[] empty = new boolean[10];

    private static int encode_height(int height, boolean right) {
        switch (height) {
            case 1:
                return 2 + (right ? 1 : 0);
            case 2:
                return 4;
            case 3:
                return 5 + (right ? 1 : 0);
            case 4:
                return 7;
            case 5:
                return 8 + (right ? 1 : 0);
            default:
                return (right ? 1 : 0);
        }
    }

    private void checkBayCompl() {
        outer: for (int i = 0; i < 6; i++) {
            bays_compl[i] = false;
            for (int j = 0; j < 10; j++) {
                if (!branches_compl[i][j]) {
                    continue outer;
                }
            }
            bays_compl[i] = true;
        }
    }

    private void createResponse() {
        // Reef
        response_values[0] = (bay == 'a' ? 1 : 0) + (bays_compl[0] ? 2 : 0);
        response_values[1] = (bay == 'b' ? 1 : 0) + (bays_compl[1] ? 2 : 0);
        response_values[2] = (bay == 'c' ? 1 : 0) + (bays_compl[2] ? 2 : 0);
        response_values[3] = (bay == 'd' ? 1 : 0) + (bays_compl[3] ? 2 : 0);
        response_values[4] = (bay == 'e' ? 1 : 0) + (bays_compl[4] ? 2 : 0);
        response_values[5] = (bay == 'f' ? 1 : 0) + (bays_compl[5] ? 2 : 0);
        // Branches
        boolean[] branches;
        if (hasReefLocation.getAsBoolean()) {
            branches = branches_compl[bay - 'a'];
        } else {
            branches = empty;
        }
        response_values[6] = ((!right && height == 0) ? 1 : 0) + (branches[0] ? 2 : 0);
        response_values[7] = ((right && height == 0) ? 1 : 0) + (branches[1] ? 2 : 0);
        response_values[8] = ((!right && height == 1) ? 1 : 0) + (branches[2] ? 2 : 0);
        response_values[9] = ((right && height == 1) ? 1 : 0) + (branches[3] ? 2 : 0);
        response_values[10] = ((height == 2) ? 1 : 0) + (branches[4] ? 2 : 0);
        response_values[11] = ((!right && height == 3) ? 1 : 0) + (branches[5] ? 2 : 0);
        response_values[12] = ((right && height == 3) ? 1 : 0) + (branches[6] ? 2 : 0);
        response_values[13] = ((height == 4) ? 1 : 0) + (branches[7] ? 2 : 0);
        response_values[14] = ((!right && height == 5) ? 1 : 0) + (branches[8] ? 2 : 0);
        response_values[15] = ((right && height == 5) ? 1 : 0) + (branches[9] ? 2 : 0);
        // Feeder Station
        response_values[16] = (fdr == 'l') ? 1 : 0;
        response_values[17] = (fdr == 'f') ? 1 : 0;
        response_values[18] = (fdr == 'r') ? 1 : 0;
        // Confirmation
        response_values[19] = (hasReefLocation.getAsBoolean() ? 0 : 2);
        response_values[20] = (hasReefLocation.getAsBoolean() ? 0 : 2);

        response.set(response_values);
    }

    private void checkReefLocation() {
        if (bay >= 'a' && bay <= 'f' && height >= 0 && height <= 5) {
            currentLocation =
                ScoringLocation.CoralLocation.fromInt(2 * (int) (bay - 'a') + (right ? 1 : 0));
            currentHeight = ScoringLocation.ElevatorHeight.fromInt(height);
        }
    }

    public Trigger hasReefLocation() {
        return hasReefLocation;
    }

    public ScoringLocation.ElevatorHeight getDesiredHeight() {
        return currentHeight;
    }

    public ScoringLocation.CoralLocation getDesiredLocation() {
        return currentLocation;
    }

}
