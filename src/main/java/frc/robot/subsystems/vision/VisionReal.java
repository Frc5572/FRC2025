package frc.robot.subsystems.vision;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.util.stream.Stream;
import org.apache.http.HttpEntity;
import org.apache.http.client.methods.CloseableHttpResponse;
import org.apache.http.client.methods.HttpGet;
import org.apache.http.client.methods.HttpPost;
import org.apache.http.entity.mime.MultipartEntityBuilder;
import org.apache.http.entity.mime.content.FileBody;
import org.apache.http.impl.client.CloseableHttpClient;
import org.apache.http.impl.client.HttpClients;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/** PhotonVision-attached implementation */
public class VisionReal implements VisionIO {

    protected final PhotonCamera[] cameras;
    private final String[] coprocessorNames = new String[] {"orangepi2.local", "orangepi3.local"};
    String tempDir = System.getProperty("java.io.tmpdir");

    /** PhotonVision-attached implementation */
    public VisionReal(Constants.Vision.CameraConstants[] constants) {
        cameras = Stream.of(constants).map((consts) -> new PhotonCamera(consts.name()))
            .toArray(PhotonCamera[]::new);
        for (String hostname : coprocessorNames) {
            createSettingsUploadThread(hostname);
        }

    }

    @Override
    public void updateInputs(CameraInputs[] inputs) {
        for (int i = 0; i < cameras.length; i++) {
            inputs[i].results =
                cameras[i].getAllUnreadResults().toArray(PhotonPipelineResult[]::new);
            inputs[i].cameraMatrix = cameras[i].getCameraMatrix();
            inputs[i].distCoeffs = cameras[i].getDistCoeffs();
        }
    }

    /**
     * Upload current April Tags Field to PV Co-Processors
     *
     * @param hostname Hostname of Co-Processor
     * @return True when completed
     * @throws IOException IOException for HTTP request
     */
    public boolean uploadAprilTagMap(String hostname) throws IOException {
        String tempFile = tempDir + "/" + hostname + "-pv-tags-upload.json";
        try (final CloseableHttpClient httpClient = HttpClients.createDefault()) {
            HttpPost postReq =
                new HttpPost("http://" + hostname + ":5800/api/settings/aprilTagFieldLayout");
            Constants.Vision.fieldLayout.serialize(tempFile);
            HttpEntity entity = MultipartEntityBuilder.create()
                .addPart("data", new FileBody(new File(tempFile))).build();
            postReq.setEntity(entity);
            try (CloseableHttpResponse response = httpClient.execute(postReq)) {
                SmartDashboard.putString("uploadSettings/" + hostname + "/AprilTags/status",
                    response.getStatusLine().getStatusCode() + ": "
                        + response.getStatusLine().getReasonPhrase());
                var ent = response.getEntity();
                if (ent != null) {
                    try (InputStream stream = ent.getContent()) {
                        String text = new String(stream.readAllBytes(), StandardCharsets.UTF_8);
                        SmartDashboard
                            .putString("uploadSettings/" + hostname + "/AprilTags/content", text);
                    }
                } else {
                    SmartDashboard.putString("uploadSettings/" + hostname + "/AprilTags/content",
                        "null");
                }
                return response.getStatusLine().getStatusCode() == 200;
            }
        }
    }

    /**
     * Check that PV has started on Co-Processor
     *
     * @param hostname Hostname of Co-Processor
     * @return True when completed
     * @throws IOException IOException for HTTP request
     */
    public boolean waitForPV(String hostname) throws IOException {
        try (final CloseableHttpClient httpClient = HttpClients.createDefault()) {
            HttpGet getReq = new HttpGet("http://" + hostname + ":5800");
            try (CloseableHttpResponse response = httpClient.execute(getReq)) {
                SmartDashboard.putString("uploadSettings/" + hostname + "/status",
                    response.getStatusLine().getStatusCode() + ": "
                        + response.getStatusLine().getReasonPhrase());
                if (response.getStatusLine().getStatusCode() == 200) {
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * Create a thread to upload PV settings/April Tag field
     *
     * @param hostname hostname of the PV Co-Processor
     */
    protected void createSettingsUploadThread(String hostname) {
        System.out.println("Uploading settings for PV: " + hostname);
        new Thread(() -> {
            Timer timer = new Timer();
            boolean run = true;
            int counter = 0;
            timer.start();
            while (run) {
                if (counter >= 12) {
                    run = false;
                }
                if (timer.advanceIfElapsed(5.0)) {
                    try {
                        if (!waitForPV(hostname)) {
                            continue;
                        }
                        if (uploadAprilTagMap(hostname)) {
                            run = false;
                        }
                    } catch (Exception e) {
                        e.printStackTrace();
                    }
                    counter++;
                }
                Thread.yield();
            }
        }).start();
    }
}
