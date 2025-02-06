package frc.tools.choreo;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.List;
import java.util.zip.ZipEntry;
import java.util.zip.ZipInputStream;
import org.apache.http.client.methods.HttpGet;
import org.apache.http.impl.client.CloseableHttpClient;
import org.apache.http.impl.client.HttpClients;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;
import frc.robot.Constants;
import frc.tools.OS;

public class Project {

    public final List<Trajectory> trajectories = new ArrayList<>();
    private final File file;

    public void save() throws IOException {
        file.createNewFile();
        FileOutputStream fos = new FileOutputStream(file);
        fos.write(asJSON().getBytes());
        fos.close();
        for (File f : file.getParentFile().listFiles()) {
            if (f.getName().endsWith(".traj")) {
                f.delete();
            }
        }
        for (Trajectory traj : trajectories) {
            traj.save(file.getParentFile());
        }
    }

    public void generateTrajectories() throws IOException, InterruptedException {
        save();
        String choreo = downloadChoreoStandalone();
        Runtime r = Runtime.getRuntime();
        Process p = r.exec(new String[] {"./" + choreo, "--chor", file.getAbsolutePath(),
            "--generate", "--all-trajectory"});
        p.waitFor();
        BufferedReader b = new BufferedReader(new InputStreamReader(p.getInputStream()));
        String line = "";
        while ((line = b.readLine()) != null) {
            System.out.println(line);
        }
        b.close();

    }

    public Project(File file) {
        this.file = file;
    }

    private static String downloadChoreoStandalone() throws IOException {
        File f = new File(".");
        for (File file : f.listFiles()) {
            switch (file.getName()) {
                case "choreo-cli":
                    return "choreo-cli";
                case "choreo-cli.exe":
                    return "choreo-cli.exe";
                default:
                    break;
            }
        }
        try (final CloseableHttpClient httpClient = HttpClients.createDefault()) {
            String zipUrl;
            if (OS.isUnix()) {
                zipUrl =
                    "https://github.com/SleipnirGroup/Choreo/releases/download/v2025.0.2/Choreo-v2025.0.2-Linux-x86_64-standalone.zip";
            } else if (OS.isWindows()) {
                zipUrl =
                    "https://github.com/SleipnirGroup/Choreo/releases/download/v2025.0.2/Choreo-v2025.0.2-Windows-x86_64-standalone.zip";
            } else {
                return null;
            }
            final HttpGet httpGet = new HttpGet(zipUrl);
            httpGet.addHeader("User-Agent", "frc5572-2025");

            var response = httpClient.execute(httpGet);
            InputStream is = response.getEntity().getContent();
            ZipInputStream zis = new ZipInputStream(is);
            ZipEntry entry;
            while ((entry = zis.getNextEntry()) != null) {
                switch (entry.getName()) {
                    case "choreo-cli":
                        FileOutputStream fos = new FileOutputStream("choreo-cli");
                        zis.transferTo(fos);
                        return "choreo-cli";
                    case "choreo-cli.exe":
                        FileOutputStream fos2 = new FileOutputStream("choreo-cli.exe");
                        zis.transferTo(fos2);
                        return "choreo-cli.exe";
                    default:
                        break;
                }
            }
        }
        return null;
    }

    private String asJSON() {
        ObjectMapper mapper = new ObjectMapper();
        ObjectNode node = mapper.createObjectNode();
        node.put("name", "test");
        node.put("version", 1);
        node.put("type", "Swerve");
        ObjectNode vars = mapper.createObjectNode();
        vars.putObject("expressions");
        vars.putObject("poses");
        node.set("variables", vars);
        ObjectNode config = mapper.createObjectNode();
        config.set("frontLeft", translation2d(mapper, Constants.Swerve.wheelBase.in(Meters) / 2.0,
            Constants.Swerve.trackWidth.in(Meters) / 2.0));
        config.set("backLeft", translation2d(mapper, -Constants.Swerve.wheelBase.in(Meters) / 2.0,
            Constants.Swerve.trackWidth.in(Meters) / 2.0));
        config.set("mass", units(mapper, Constants.Swerve.robotMass.in(Kilograms), "kg"));
        config.set("inertia",
            units(mapper, Constants.Swerve.robotMOI.in(KilogramSquareMeters), "kg m ^ 2"));
        config.set("gearing", units(mapper, Constants.Swerve.ModuleConstants.driveReduction, ""));
        config.set("radius", units(mapper, Constants.Swerve.wheelDiameter.in(Meters) / 2.0, "m"));
        config.set("vmax", units(mapper,
            Constants.Swerve.ModuleConstants.driveMotor.freeSpeedRadPerSec, "rad / s"));
        config.set("tmax", units(mapper, Constants.Swerve.ModuleConstants.driveMotor.KtNMPerAmp
            * Constants.Swerve.ModuleConstants.driveCurrentLimit.in(Amps), "N * m"));
        config.set("cof", units(mapper, Constants.Swerve.ModuleConstants.wheelCoeffFriction, ""));
        ObjectNode bumper = mapper.createObjectNode();
        bumper.set("front", units(mapper, Constants.Swerve.bumperFront.in(Meters), "m"));
        bumper.set("side", units(mapper, Constants.Swerve.bumperRight.in(Meters), "m"));
        bumper.set("back", units(mapper, Constants.Swerve.bumperFront.in(Meters), "m"));
        config.set("bumper", bumper);
        config.set("differentialTrackWidth",
            units(mapper, Constants.Swerve.trackWidth.in(Meters), "m"));
        node.set("config", config);
        node.putArray("generationFeatures");
        return node.toPrettyString();
    }

    private static ObjectNode translation2d(ObjectMapper mapper, double x, double y) {
        ObjectNode node = mapper.createObjectNode();
        node.set("x", units(mapper, x, "m"));
        node.set("y", units(mapper, y, "m"));
        return node;
    }

    public static ObjectNode units(ObjectMapper mapper, double value, String suffix) {
        ObjectNode node = mapper.createObjectNode();
        node.put("exp", Double.toString(value) + " " + suffix);
        node.put("val", value);
        return node;
    }
}
