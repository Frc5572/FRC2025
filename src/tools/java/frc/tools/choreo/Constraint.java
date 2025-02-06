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
            // TODO Auto-generated method stub
            return null;
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
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'toJSON'");
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
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'toJSON'");
        }

    }

}
