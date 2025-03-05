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
        for (String copro : coprocessorNames) {
            SmartDashboard.putString("uploadSettings/" + copro + "/test", copro);
            new Thread(() -> {
                SmartDashboard.putString("uploadSettings/" + copro + "/test1", copro);
                Timer timer = new Timer();
                boolean run = true;
                timer.start();
                while (run) {
                    SmartDashboard.putNumber("uploadSettings/" + copro + "/test2",
                        Timer.getFPGATimestamp());
                    if (timer.advanceIfElapsed(5.0)) {
                        SmartDashboard.putString("uploadSettings/" + copro + "/test3", copro);
                        try {
                            if (!waitForPV(copro)) {
                                SmartDashboard.putNumber("uploadSettings/" + copro + "/test4",
                                    Timer.getFPGATimestamp());
                            }
                            if (uploadAprilTagMap(copro)) {
                                run = false;
                            }
                        } catch (Exception e) {
                            // SmartDashboard.putString("uploadSettings/" + copro + "/error",
                            // e);
                            e.printStackTrace();
                        }
                    }
                    Thread.yield();
                }
            }).start();
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

    public boolean uploadAprilTagMap(String hostname) throws IOException {
        SmartDashboard.putNumber("uploadSettings/" + hostname + "/test5", Timer.getFPGATimestamp());
        String tempFile = tempDir + "/" + hostname + "-pv-tags-upload.json";
        try (final CloseableHttpClient httpClient = HttpClients.createDefault()) {
            HttpPost postReq =
                new HttpPost("http://" + hostname + ":5800/api/settings/aprilTagFieldLayout");
            Constants.Vision.fieldLayout.serialize(tempFile);
            HttpEntity entity = MultipartEntityBuilder.create()
                .addPart("data", new FileBody(new File(tempFile))).build();
            postReq.setEntity(entity);
            SmartDashboard.putNumber("uploadSettings/" + hostname + "/test6",
                Timer.getFPGATimestamp());

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
     * Upload Settings
     *
     * @param ip Camera IP
     * @param file Camera settings file
     */
    // public boolean uploadSettings(String ip, File file) throws IOException {
    // try (final CloseableHttpClient httpClient = HttpClients.createDefault()) {
    // HttpPost postReq = new HttpPost("http://" + ip + "/api/settings");
    // HttpEntity entity =
    // MultipartEntityBuilder.create().addPart("data", new FileBody(file)).build();
    // postReq.setEntity(entity);
    // try (CloseableHttpResponse response = httpClient.execute(postReq)) {
    // SmartDashboard.putString("uploadSettings/" + this.name + "/status",
    // response.getStatusLine().getStatusCode() + ": "
    // + response.getStatusLine().getReasonPhrase());
    // var ent = response.getEntity();
    // if (ent != null) {
    // try (InputStream stream = ent.getContent()) {
    // String text = new String(stream.readAllBytes(), StandardCharsets.UTF_8);
    // SmartDashboard.putString("uploadSettings/" + this.name + "/content", text);
    // }
    // } else {
    // SmartDashboard.putString("uploadSettings/" + this.name + "/content", "null");
    // }
    // return response.getStatusLine().getStatusCode() == 200;
    // }
    // }
    // }

}
