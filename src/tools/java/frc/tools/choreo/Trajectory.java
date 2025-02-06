package frc.tools.choreo;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;

public class Trajectory {

    private final String name;

    public final List<Waypoint> waypoints = new ArrayList<>();
    public final List<Constraint> constraints = new ArrayList<>();

    public Trajectory(String name) {
        this.name = name;
    }

    public void save(File folder) throws IOException {
        File file = new File(folder, name + ".traj");
        file.createNewFile();
        FileOutputStream fos = new FileOutputStream(file);
        fos.write(asJSON().getBytes());
        fos.close();
    }

    private String asJSON() {
        ObjectMapper mapper = new ObjectMapper();
        ObjectNode node = mapper.createObjectNode();
        node.put("name", name);
        node.put("version", 1);
        ObjectNode snapshotNode = mapper.createObjectNode();
        snapshotNode.putArray("waypoints");
        snapshotNode.putArray("constraints");
        snapshotNode.put("targetDt", 0.05);
        node.set("snapshot", snapshotNode);
        ObjectNode paramsNode = mapper.createObjectNode();
        ArrayNode waypointsNode = mapper.createArrayNode();
        for (Waypoint w : waypoints) {
            waypointsNode.add(w.toJSON(mapper));
        }
        paramsNode.set("waypoints", waypointsNode);
        ArrayNode constraintsNode = mapper.createArrayNode();
        for (Constraint c : constraints) {
            constraintsNode.add(c.toJSON(mapper));
        }
        paramsNode.set("constraints", constraintsNode);
        paramsNode.set("targetDt", Project.units(mapper, 0.05, "s"));
        node.set("params", paramsNode);
        ObjectNode trajectory = mapper.createObjectNode();
        trajectory.put("sampleType", (String) null);
        trajectory.putArray("waypoints");
        trajectory.putArray("samples");
        trajectory.putArray("splits");
        node.set("trajectory", trajectory);
        node.putArray("events");
        return node.toPrettyString();
    }

}
