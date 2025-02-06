package frc.tools.choreo;

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
import frc.tools.OS;

public class Project {

    public final List<Trajectory> trajectories = new ArrayList<>();
    private final File file;

    public void save() throws IOException {
        file.createNewFile();
        FileOutputStream fos = new FileOutputStream(file);
        // TODO
        fos.close();
    }

    public void generateTrajectories() throws IOException, InterruptedException {
        save();
        String choreo = downloadChoreoStandalone();
        Runtime r = Runtime.getRuntime();
        Process p = r.exec(new String[] {choreo, "--chor", file.getAbsolutePath(), "--generate"});
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

}
