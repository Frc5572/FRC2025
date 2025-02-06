package frc.tools.choreo;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;

public sealed interface Constraint {

    public ObjectNode toJSON(ObjectMapper mapper);

    public static final class StopPoint implements Constraint {

        public final WaypointIdx idx;

        public StopPoint(WaypointIdx idx) {
            this.idx = idx;
        }

        public StopPoint(int idx) {
            this(new WaypointIdx.Idx(idx));
        }

        public static StopPoint first() {
            return new StopPoint(new WaypointIdx.First());
        }

        public static StopPoint last() {
            return new StopPoint(new WaypointIdx.Last());
        }

        @Override
        public ObjectNode toJSON(ObjectMapper mapper) {
            ObjectNode node = mapper.createObjectNode();
            idx.addToObject(node, "from");
            node.put("to", (String) null);
            ObjectNode data = mapper.createObjectNode();
            data.put("type", "StopPoint");
            data.putObject("props");
            node.set("data", data);
            node.put("enabled", true);
            return node;
        }

    }

    public static final class KeepInRectangle implements Constraint {

        public final WaypointIdx from;
        public final WaypointIdx to;

        public final double x;
        public final double y;
        public final double w;
        public final double h;

        public KeepInRectangle(WaypointIdx from, WaypointIdx to, double x, double y, double w,
            double h) {
            this.from = from;
            this.to = to;
            this.x = x;
            this.y = y;
            this.w = w;
            this.h = h;
        }

        @Override
        public ObjectNode toJSON(ObjectMapper mapper) {
            ObjectNode node = mapper.createObjectNode();
            from.addToObject(node, "from");
            to.addToObject(node, "to");
            ObjectNode data = mapper.createObjectNode();
            data.put("type", "KeepInRectangle");
            ObjectNode props = mapper.createObjectNode();
            props.set("x", Project.units(mapper, x, "m"));
            props.set("y", Project.units(mapper, y, "m"));
            props.set("w", Project.units(mapper, w, "m"));
            props.set("h", Project.units(mapper, h, "m"));
            data.set("props", props);
            node.set("data", data);
            node.put("enabled", true);
            return node;
        }

    }

    public static final class KeepOutCircle implements Constraint {

        public final WaypointIdx from;
        public final WaypointIdx to;

        public final double x;
        public final double y;
        public final double r;

        public KeepOutCircle(WaypointIdx from, WaypointIdx to, double x, double y, double r) {
            this.from = from;
            this.to = to;
            this.x = x;
            this.y = y;
            this.r = r;
        }

        @Override
        public ObjectNode toJSON(ObjectMapper mapper) {
            ObjectNode node = mapper.createObjectNode();
            from.addToObject(node, "from");
            to.addToObject(node, "to");
            ObjectNode data = mapper.createObjectNode();
            data.put("type", "KeepOutCircle");
            ObjectNode props = mapper.createObjectNode();
            props.set("x", Project.units(mapper, x, "m"));
            props.set("y", Project.units(mapper, y, "m"));
            props.set("r", Project.units(mapper, r, "m"));
            data.set("props", props);
            node.set("data", data);
            node.put("enabled", true);
            return node;
        }

    }

}
