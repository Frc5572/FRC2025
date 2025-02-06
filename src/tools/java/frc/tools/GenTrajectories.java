package frc.tools;

import static edu.wpi.first.units.Units.Meters;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.lib.math.Circle;
import frc.lib.util.ScoringLocation;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.tools.choreo.Constraint;
import frc.tools.choreo.Project;
import frc.tools.choreo.Trajectory;
import frc.tools.choreo.Waypoint;
import frc.tools.choreo.WaypointIdx;

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
        new Translation2d(2.467679977416992, 0.6873600482940674);
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

    public static void main(String[] args) throws IOException, InterruptedException {
        Files.createDirectories(Path.of("test"));
        Project project = new Project(new File("test/test.chor"));
        // RightFeeder to Reef
        for (int i = 0; i < 10; i++) {
            Pose2d rightFeederPose = RobotState.constrain(
                RIGHT_FEEDER_POSE_CLOSE.interpolate(RIGHT_FEEDER_POSE_FAR, (double) i / 9.0),
                (_p) -> {
                }, Units.inchesToMeters(0.5));
            Rotation2d startRotation = driveCircle.getAngle(rightFeederPose.getTranslation());
            for (ScoringLocation.CoralLocation location : ScoringLocation.CoralLocation.values()) {
                Pose2d reefPose = RobotState.constrain(location.driveLocation, (_p) -> {
                }, Units.inchesToMeters(0.5));
                Rotation2d endRotation = driveCircle.getAngle(reefPose.getTranslation());

                Pose2d p = new Pose2d(
                    reefPose.getTranslation().minus(new Translation2d(
                        Constants.Swerve.bumperFront.in(Meters), reefPose.getRotation())),
                    reefPose.getRotation());
                {
                    Trajectory trajectory =
                        new Trajectory("Path_CCW_RF" + i + "_to_C" + location.toString());
                    trajectory.waypoints.add(new Waypoint.RegularWaypoint(rightFeederPose));
                    trajectory.waypoints.add(RIGHT_ICECREAM_AVOID);

                    Rotation2d next = startRotation;
                    while (Math.abs(next.getDegrees() - endRotation.getDegrees()) > 15.0) {
                        next = next.plus(Rotation2d.fromDegrees(15));
                        trajectory.waypoints
                            .add(new Waypoint.EmptyWaypoint(driveCircle.getVertex(next, 0.1)));
                    }

                    trajectory.waypoints.add(new Waypoint.RegularWaypoint(p));
                    trajectory.waypoints.add(new Waypoint.RegularWaypoint(reefPose));
                    addCommonConstraints(trajectory, new WaypointIdx.First(),
                        new WaypointIdx.Idx(trajectory.waypoints.size() - 2));
                    project.trajectories.add(trajectory);
                }
                {
                    Trajectory trajectory =
                        new Trajectory("Path_CW_RF" + i + "_to_C" + location.toString());
                    trajectory.waypoints.add(new Waypoint.RegularWaypoint(rightFeederPose));
                    trajectory.waypoints.add(RIGHT_ICECREAM_AVOID);

                    Rotation2d next = startRotation;
                    while (Math.abs(next.getDegrees() - endRotation.getDegrees()) > 15.0) {
                        next = next.minus(Rotation2d.fromDegrees(15));
                        trajectory.waypoints
                            .add(new Waypoint.EmptyWaypoint(driveCircle.getVertex(next, 0.1)));
                    }

                    trajectory.waypoints.add(new Waypoint.RegularWaypoint(p));
                    trajectory.waypoints.add(new Waypoint.RegularWaypoint(reefPose));
                    addCommonConstraints(trajectory, new WaypointIdx.First(),
                        new WaypointIdx.Idx(trajectory.waypoints.size() - 2));
                    project.trajectories.add(trajectory);
                }
            }
        }
        project.save();
    }

    private static void addCommonConstraints(Trajectory trajectory, WaypointIdx from,
        WaypointIdx to) {
        trajectory.constraints.add(Constraint.StopPoint.first());
        trajectory.constraints.add(Constraint.StopPoint.last());
        trajectory.constraints.add(
            new Constraint.KeepInRectangle(new WaypointIdx.First(), new WaypointIdx.Last(), 0, 0,
                FieldConstants.fieldLength.in(Meters) / 2.0, FieldConstants.fieldWidth.in(Meters)));
        trajectory.constraints
            .add(new Constraint.KeepOutCircle(from, to, reefCircle.getCenter().getX(),
                reefCircle.getCenter().getY(), reefCircle.getRadius() + Units.inchesToMeters(3)));
        trajectory.constraints.add(new Constraint.KeepOutCircle(from, to,
            iceCreamCircles[0].getCenter().getX(), iceCreamCircles[0].getCenter().getY(),
            iceCreamCircles[0].getRadius() + Units.inchesToMeters(3)));
        trajectory.constraints.add(new Constraint.KeepOutCircle(from, to,
            iceCreamCircles[1].getCenter().getX(), iceCreamCircles[1].getCenter().getY(),
            iceCreamCircles[1].getRadius() + Units.inchesToMeters(3)));
        trajectory.constraints.add(new Constraint.KeepOutCircle(from, to,
            iceCreamCircles[2].getCenter().getX(), iceCreamCircles[2].getCenter().getY(),
            iceCreamCircles[2].getRadius() + Units.inchesToMeters(3)));
    }

}
