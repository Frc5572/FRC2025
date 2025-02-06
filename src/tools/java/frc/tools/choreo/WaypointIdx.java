package frc.tools.choreo;

import com.fasterxml.jackson.databind.node.ObjectNode;

public sealed interface WaypointIdx {

    public void addToObject(ObjectNode node, String key);

    public static final class First implements WaypointIdx {

        @Override
        public void addToObject(ObjectNode node, String key) {
            node.put(key, "first");
        }

    }

    public static final class Last implements WaypointIdx {
        @Override
        public void addToObject(ObjectNode node, String key) {
            node.put(key, "last");
        }
    }

    public static final class Idx implements WaypointIdx {
        public final int id;

        public Idx(int id) {
            this.id = id;
        }

        @Override
        public void addToObject(ObjectNode node, String key) {
            node.put(key, id);
        }

    }

}
