package frc.tools;

import static edu.wpi.first.units.Units.Meters;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.lib.math.Circle;
import frc.lib.util.ScoringLocation;
import frc.robot.Constants;
import frc.robot.FieldConstants;

public class GenTrajectories {

    private static final Pose2d RIGHT_FEEDER_POSE_CLOSE =
        new Pose2d(0.5922711491584778, 1.305626630783081, Rotation2d.fromDegrees(-35));
    private static final Pose2d RIGHT_FEEDER_POSE_FAR =
        new Pose2d(1.5608888864517212, 0.6255332827568054, Rotation2d.fromDegrees(-35));

    private static final Pose2d LEFT_FEEDER_POSE_CLOSE = new Pose2d(RIGHT_FEEDER_POSE_CLOSE.getX(),
        FieldConstants.fieldWidth.in(Meters) - RIGHT_FEEDER_POSE_CLOSE.getY(),
        RIGHT_FEEDER_POSE_CLOSE.getRotation().unaryMinus());
    private static final Pose2d LEFT_FEEDER_POSE_FAR = new Pose2d(RIGHT_FEEDER_POSE_FAR.getX(),
        FieldConstants.fieldWidth.in(Meters) - RIGHT_FEEDER_POSE_FAR.getY(),
        RIGHT_FEEDER_POSE_FAR.getRotation().unaryMinus());

    private static final Translation2d avoidIceCreamTranslation =
        new Translation2d(1.9730666875839233, 1.7384133338928223);
    private static final Waypoint.EmptyWaypoint RIGHT_ICECREAM_AVOID =
        new Waypoint.EmptyWaypoint(avoidIceCreamTranslation);
    private static final Waypoint.EmptyWaypoint LEFT_ICECREAM_AVOID =
        new Waypoint.EmptyWaypoint(new Translation2d(avoidIceCreamTranslation.getX(),
            FieldConstants.fieldWidth.in(Meters) - avoidIceCreamTranslation.getY()));

    private static final double rightMost = 0.378226637840271;
    private static final double leftMost = 7.653164386749268;

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

    private static final Waypoint.EmptyWaypoint[] emptyWaypoints = new Waypoint.EmptyWaypoint[] {
        // @formatter:off
        new Waypoint.EmptyWaypoint(new Translation2d(6.877982139587402, 5.28314208984375)),
        new Waypoint.EmptyWaypoint(new Translation2d(4.549177646636963, 6.66393804550170)),
        new Waypoint.EmptyWaypoint(new Translation2d(2.261591196060181, 5.44801330566406)),
        new Waypoint.EmptyWaypoint(new Translation2d(2.282200098037720, 3.09859991073608)),
        new Waypoint.EmptyWaypoint(new Translation2d(4.549177646636963, 1.40867114067078)),
        new Waypoint.EmptyWaypoint(new Translation2d(6.836764812469482, 2.99555563926697)),
        // @formatter:on
    };

    public static void main(String[] args) {
        // RightFeeder to Reef
        for (int i = 0; i < 10; i++) {
            Pose2d rightFeederPose =
                RIGHT_FEEDER_POSE_CLOSE.interpolate(RIGHT_FEEDER_POSE_FAR, (double) i / 9.0);
            Rotation2d startRotation = driveCircle.getAngle(rightFeederPose.getTranslation());
            for (ScoringLocation.CoralLocation location : ScoringLocation.CoralLocation.values()) {
                Pose2d reefPose = location.driveLocation;
                Rotation2d endRotation = driveCircle.getAngle(reefPose.getTranslation());

                Pose2d p = new Pose2d(
                    reefPose.getTranslation().minus(new Translation2d(
                        Constants.Swerve.bumperFront.in(Meters), reefPose.getRotation())),
                    reefPose.getRotation());
                {
                    List<Waypoint> waypoints = new ArrayList<>();
                    waypoints.add(new Waypoint.RegularWaypoint(rightFeederPose));
                    waypoints.add(RIGHT_ICECREAM_AVOID);

                    Rotation2d next = startRotation;
                    while (Math.abs(next.getDegrees() - endRotation.getDegrees()) > 15.0) {
                        next = next.plus(Rotation2d.fromDegrees(15));
                        waypoints.add(new Waypoint.EmptyWaypoint(driveCircle.getVertex(next, 0.5)));
                    }

                    waypoints.add(new Waypoint.RegularWaypoint(p));
                    waypoints.add(new Waypoint.RegularWaypoint(reefPose));
                    createTrajectory("Path_CCW_RF" + i + "_to_C" + location.toString(),
                        waypoints.toArray(Waypoint[]::new));
                }
                {
                    List<Waypoint> waypoints = new ArrayList<>();
                    waypoints.add(new Waypoint.RegularWaypoint(rightFeederPose));
                    waypoints.add(RIGHT_ICECREAM_AVOID);

                    Rotation2d next = startRotation;
                    while (Math.abs(next.getDegrees() - endRotation.getDegrees()) > 15.0) {
                        next = next.minus(Rotation2d.fromDegrees(15));
                        waypoints.add(new Waypoint.EmptyWaypoint(driveCircle.getVertex(next, 0.5)));
                    }

                    waypoints.add(new Waypoint.RegularWaypoint(p));
                    waypoints.add(new Waypoint.RegularWaypoint(reefPose));
                    createTrajectory("Path_CW_RF" + i + "_to_C" + location.toString(),
                        waypoints.toArray(Waypoint[]::new));
                }
            }
        }
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
