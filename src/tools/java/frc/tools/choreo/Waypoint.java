package frc.tools.choreo;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public sealed interface Waypoint {

    public ObjectNode toJSON(ObjectMapper mapper);

    public static final class RegularWaypoint implements Waypoint {
        private final Pose2d pose;

        public RegularWaypoint(Pose2d pose) {
            this.pose = pose;
        }

        @Override
        public ObjectNode toJSON(ObjectMapper mapper) {
            ObjectNode node = mapper.createObjectNode();
            node.set("x", units(mapper, pose.getX(), "m"));
            node.set("y", units(mapper, pose.getY(), "m"));
            node.set("heading", units(mapper, pose.getRotation().getRadians(), "rad"));
            node.put("intervals", 40);
            node.put("split", false);
            node.put("fixTranslation", true);
            node.put("fixHeading", true);
            node.put("overrideIntervals", false);
            return node;
        }


        private static ObjectNode units(ObjectMapper mapper, double value, String suffix) {
            ObjectNode node = mapper.createObjectNode();
            node.put("exp", Double.toString(value) + " " + suffix);
            node.put("val", value);
            return node;
        }
    }

    public static final class EmptyWaypoint implements Waypoint {
        private final Translation2d pos;

        public EmptyWaypoint(Translation2d pos) {
            this.pos = pos;
        }

        @Override
        public ObjectNode toJSON(ObjectMapper mapper) {
            ObjectNode node = mapper.createObjectNode();
            node.set("x", units(mapper, pos.getX(), "m"));
            node.set("y", units(mapper, pos.getY(), "m"));
            node.set("heading", units(mapper, 0, "rad"));
            node.put("intervals", 40);
            node.put("split", false);
            node.put("fixTranslation", false);
            node.put("fixHeading", false);
            node.put("overrideIntervals", false);
            return node;
        }


        private static ObjectNode units(ObjectMapper mapper, double value, String suffix) {
            ObjectNode node = mapper.createObjectNode();
            node.put("exp", Double.toString(value) + " " + suffix);
            node.put("val", value);
            return node;
        }
    }

}
