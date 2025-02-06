package frc.tools;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Random;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.lib.math.Circle;
import frc.robot.Constants;
import frc.robot.FieldConstants;

public class GenTrajectories2 {

    private static final Circle reefCircle = new Circle("reef", FieldConstants.Reef.center,
        FieldConstants.Reef.circumscribedRadius.in(Meters));

    private static final Circle driveCircle = new Circle("drive", FieldConstants.Reef.center,
        FieldConstants.Reef.circumscribedRadius.in(Meters)
            + 2.0 * Math.hypot(Constants.Swerve.bumperFront.in(Meters),
                Constants.Swerve.bumperRight.in(Meters)));

    private static final Circle[] iceCreamCircles = new Circle[] {
        // @formatter:off
        new Circle("iceCream1", FieldConstants.StagingPositions.leftIceCream.getTranslation(), 0.3),
        new Circle("iceCream2", FieldConstants.StagingPositions.middleIceCream.getTranslation(), 0.3),
        new Circle("iceCream3", FieldConstants.StagingPositions.rightIceCream.getTranslation(), 0.3),
        // @formatter:on
    };

    public static void main(String[] args) throws IOException, InterruptedException {
        // Generate 1000 poses
        int numPoses = 1000;
        Random rand = new Random(5572);
        Pose2d[] poses = new Pose2d[numPoses];
        for (int i = 0; i < numPoses; i++) {
            double x = rand.nextDouble() * FieldConstants.fieldLength.in(Meters) / 2.0;
            double y = rand.nextDouble() * FieldConstants.fieldWidth.in(Meters);
            Rotation2d t = new Rotation2d(Rotations.of(rand.nextDouble()));
            poses[i] = frc.robot.RobotState.constrain(new Pose2d(x, y, t), (_p) -> {
            });
        }
        for (int i = 0; i < numPoses; i++) {
            Pose2d pose1 = poses[i];
            for (int j = i + 1; j < numPoses; j++) {
                Pose2d pose2 = poses[j];
                generateTrajectory(pose1, pose2);
                generateTrajectory(pose2, pose1);
            }
        }
    }

    private static void generateTrajectory(Pose2d pose1, Pose2d pose2)
        throws IOException, InterruptedException {
        generateTrajectoryDirection(pose1, pose2, Rotation2d.fromDegrees(15));
        generateTrajectoryDirection(pose1, pose2, Rotation2d.fromDegrees(-15));
        System.exit(0);
    }

    private static void generateTrajectoryDirection(Pose2d pose1, Pose2d pose2,
        Rotation2d direction) throws IOException, InterruptedException {
        System.out.println("Generating for " + pose1 + " to " + pose2);
        Rotation2d startRotation = driveCircle.getAngle(pose1.getTranslation());
        Rotation2d endRotation = driveCircle.getAngle(pose2.getTranslation());

        System.exit(0);
    }

    private static sealed interface Waypoint {

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
        }
    }

    private static void createTrajectory(String name, Waypoint[] waypoints) {
        ObjectMapper mapper = new ObjectMapper();
        ObjectNode rootNode = mapper.createObjectNode();
        rootNode.put("name", name);
        rootNode.put("version", 1);
        ObjectNode snapshotNode = mapper.createObjectNode();
        snapshotNode.putArray("waypoints");
        snapshotNode.putArray("constraints");
        snapshotNode.put("targetDt", 0.05);
        rootNode.set("snapshot", snapshotNode);
        ObjectNode paramsNode = mapper.createObjectNode();
        ArrayNode waypointsNode = mapper.createArrayNode();
        for (Waypoint wp : waypoints) {
            waypointsNode.add(wp.toJSON(mapper));
        }
        paramsNode.set("waypoints", waypointsNode);
        ArrayNode constraintsNode = mapper.createArrayNode();
        constraintsNode.add(stopPoint(mapper, "first"));
        constraintsNode.add(stopPoint(mapper, "last"));
        constraintsNode.add(keepInField(mapper));
        int to = waypoints.length - 2;
        constraintsNode.add(keepOutCircle(mapper, reefCircle, 0, to));
        constraintsNode.add(keepOutCircle(mapper, iceCreamCircles[0], 0, to));
        constraintsNode.add(keepOutCircle(mapper, iceCreamCircles[1], 0, to));
        constraintsNode.add(keepOutCircle(mapper, iceCreamCircles[2], 0, to));
        paramsNode.set("constraints", constraintsNode);
        ObjectNode targetDtNode = mapper.createObjectNode();
        targetDtNode.put("exp", "0.05 s");
        targetDtNode.put("val", 0.05);
        paramsNode.set("targetDt", targetDtNode);
        rootNode.set("params", paramsNode);
        ObjectNode trajectoryNode = mapper.createObjectNode();
        trajectoryNode.put("sampleType", (String) null);
        trajectoryNode.putArray("waypoints");
        trajectoryNode.putArray("samples");
        trajectoryNode.putArray("splits");
        rootNode.set("trajectory", trajectoryNode);
        rootNode.putArray("events");

        try (FileWriter writer = new FileWriter(name + ".traj")) {
            writer.write(rootNode.toPrettyString());
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private static ObjectNode stopPoint(ObjectMapper mapper, String from) {
        ObjectNode node = mapper.createObjectNode();
        node.put("from", from);
        node.put("to", (String) null);
        ObjectNode dataNode = mapper.createObjectNode();
        dataNode.put("type", "StopPoint");
        dataNode.set("props", mapper.createObjectNode());
        node.set("data", dataNode);
        node.put("enabled", true);
        return node;
    }

    private static ObjectNode keepOutCircle(ObjectMapper mapper, Circle circle, int from, int to) {
        ObjectNode node = mapper.createObjectNode();
        node.put("from", from);
        node.put("to", to);
        ObjectNode dataNode = mapper.createObjectNode();
        dataNode.put("type", "KeepOutCircle");
        ObjectNode propsNode = mapper.createObjectNode();
        propsNode.set("x", units(mapper, circle.getCenter().getX(), "m"));
        propsNode.set("y", units(mapper, circle.getCenter().getY(), "m"));
        propsNode.set("r", units(mapper, circle.getRadius() + Units.inchesToMeters(3), "m"));
        dataNode.set("props", propsNode);
        node.set("data", dataNode);
        node.put("enabled", true);
        return node;
    }

    private static ObjectNode keepInField(ObjectMapper mapper) {
        ObjectNode node = mapper.createObjectNode();
        node.put("from", "first");
        node.put("to", "last");
        ObjectNode dataNode = mapper.createObjectNode();
        dataNode.put("type", "KeepInRectangle");
        ObjectNode propsNode = mapper.createObjectNode();
        propsNode.set("x", units(mapper, 0.0, "m"));
        propsNode.set("y", units(mapper, 0.0, "m"));
        propsNode.set("w", units(mapper, 17.548, "m"));
        propsNode.set("h", units(mapper, 8.052, "m"));
        dataNode.set("props", propsNode);
        node.set("data", dataNode);
        node.put("enabled", true);
        return node;
    }

    private static ObjectNode units(ObjectMapper mapper, double value, String suffix) {
        ObjectNode node = mapper.createObjectNode();
        node.put("exp", Double.toString(value) + " " + suffix);
        node.put("val", value);
        return node;
    }

}
