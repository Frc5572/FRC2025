package frc.lib.draw;

import java.io.File;
import java.io.IOException;
import com.fasterxml.jackson.core.exc.StreamWriteException;
import com.fasterxml.jackson.databind.DatabindException;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;

public class DrawingUtils {

    public static interface Drawable {
        public void draw();

        public ObjectNode layout(ObjectMapper om);
    }

    private static Drawable[] drawables = new Drawable[0];

    public static void addDrawable(Drawable drawable) {
        Drawable[] new_drawables = new Drawable[drawables.length + 1];
        System.arraycopy(drawables, 0, new_drawables, 0, drawables.length);
        new_drawables[drawables.length] = drawable;
        drawables = new_drawables;
    }

    public static void draw() {
        for (var d : drawables) {
            d.draw();
        }
    }

    public static void layout() {
        ObjectMapper om = new ObjectMapper();
        ObjectNode root = createRoot(om);
        try {
            om.writeValue(new File("AdvantageScopeLayout.json"), root);
        } catch (StreamWriteException e) {
            e.printStackTrace();
        } catch (DatabindException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private static ObjectNode createRoot(ObjectMapper om) {
        ObjectNode root = om.createObjectNode();
        root.set("hubs", createHubs(om));
        root.set("satellites", om.createArrayNode());
        root.put("version", "4.1.0");
        return root;
    }

    private static ArrayNode createHubs(ObjectMapper om) {
        ArrayNode hubs = om.createArrayNode();
        hubs.add(createHub(om));
        return hubs;
    }

    private static ObjectNode createHub(ObjectMapper om) {
        ObjectNode hub = om.createObjectNode();
        hub.put("x", 2587);
        hub.put("y", 212);
        hub.put("width", 1100);
        hub.put("height", 650);
        hub.set("state", createState(om));
        return hub;
    }

    private static ObjectNode createState(ObjectMapper om) {
        ObjectNode state = om.createObjectNode();
        state.set("sidebar", createSidebar(om));
        state.set("tabs", createTabs(om));
        return state;
    }

    private static ObjectNode createSidebar(ObjectMapper om) {
        ObjectNode sidebar = om.createObjectNode();
        sidebar.put("width", 300);
        sidebar.set("expanded", createExpanded(om));
        return sidebar;
    }

    private static String[] expanded = new String[] {"/AdvantageKit", "/AdvantageKit/RealOutputs",
        "/AdvantageKit/RealOutputs/FieldSimulation", "/AdvantageKit/RealOutputs/Swerve"};

    private static ArrayNode createExpanded(ObjectMapper om) {
        ArrayNode expanded = om.createArrayNode();
        for (var e : DrawingUtils.expanded) {
            expanded.add(e);
        }
        return expanded;
    }

    private static ObjectNode createTabs(ObjectMapper om) {
        ObjectNode tabsOuter = om.createObjectNode();
        tabsOuter.put("selected", 1);
        ArrayNode tabs = om.createArrayNode();
        tabs.add(createHelp(om));
        tabs.add(create3DField(om));
        tabsOuter.set("tabs", tabs);
        return tabsOuter;
    }

    private static ObjectNode createHelp(ObjectMapper om) {
        ObjectNode help = om.createObjectNode();
        help.put("type", 0);
        help.put("title", "");
        help.put("controller", (String) null);
        help.put("controllerUUID", "fc23sw0u68w4m0e33w5chwru31i5jbke");
        help.put("renderer", "#/");
        help.put("controlsHeight", 0);
        return help;
    }

    private static ObjectNode create3DField(ObjectMapper om) {
        ObjectNode field = om.createObjectNode();
        field.put("type", 3);
        field.put("title", "3D Field");
        field.set("controller", create3DFieldController(om));
        field.put("controllerUUID", "vm7sys826u2ladjvooxorxbzm15exzys");
        field.set("renderer", create3DFieldRenderer(om));
        field.put("controlsHeight", 200);
        return field;
    }

    private static ObjectNode create3DFieldController(ObjectMapper om) {
        ObjectNode controller = om.createObjectNode();
        ArrayNode sources = om.createArrayNode();
        sources.add(createRobotSource(om));
        sources.add(createCoralSource(om));
        sources.add(createAlgaeSource(om));
        for (Drawable d : drawables) {
            sources.add(d.layout(om));
        }
        controller.set("sources", sources);
        controller.put("game", "2025 Field");
        controller.put("origin", "blue");
        return controller;
    }

    private static ObjectNode create3DFieldRenderer(ObjectMapper om) {
        ObjectNode renderer = om.createObjectNode();
        renderer.put("cameraIndex", -1);
        renderer.put("orbitFov", 50);
        ArrayNode cameraPosition = om.createArrayNode();
        cameraPosition.add(10.627686386379807);
        cameraPosition.add(10.321630737811434);
        cameraPosition.add(-0.1724847690458184);
        renderer.set("cameraPosition", cameraPosition);
        ArrayNode cameraTarget = om.createArrayNode();
        cameraTarget.add(3.8788295718087493);
        cameraTarget.add(-1.8239663014334084);
        cameraTarget.add(-0.06204700162981045);
        renderer.set("cameraTarget", cameraTarget);

        return renderer;
    }

    private static ObjectNode createRobotSource(ObjectMapper om) {
        ObjectNode source = om.createObjectNode();
        source.put("type", "robot");
        source.put("logKey", "NT:/AdvantageKit/RealOutputs/simulatedPose");
        source.put("logType", "Pose2d");
        source.put("visible", true);
        ObjectNode options = om.createObjectNode();
        options.put("model", "2025 KitBot");
        source.set("options", options);
        return source;
    }

    private static ObjectNode createCoralSource(ObjectMapper om) {
        ObjectNode source = om.createObjectNode();
        source.put("type", "gamePiece");
        source.put("logKey", "NT:/AdvantageKit/RealOutputs/FieldSimulation/Coral");
        source.put("logType", "Pose3d[]");
        source.put("visible", true);
        ObjectNode options = om.createObjectNode();
        options.put("variant", "Coral");
        source.set("options", options);
        return source;
    }

    private static ObjectNode createAlgaeSource(ObjectMapper om) {
        ObjectNode source = om.createObjectNode();
        source.put("type", "gamePiece");
        source.put("logKey", "NT:/AdvantageKit/RealOutputs/FieldSimulation/Algae");
        source.put("logType", "Pose3d[]");
        source.put("visible", true);
        ObjectNode options = om.createObjectNode();
        options.put("variant", "Algae");
        source.set("options", options);
        return source;
    }

}
