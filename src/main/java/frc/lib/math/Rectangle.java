package frc.lib.math;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.util.viz.Drawable;

/** Rotating Rectangle Shape */
public class Rectangle implements ConvexShape, Drawable {

    public final double width;
    public final double length;

    private final Axis[] axes;
    private final Translation2d[] vertices;

    private Pose2d pose;

    private final String name;

    /** Rotating Rectangle Shape */
    public Rectangle(String name, Pose2d pose, double length, double width) {
        this.name = name;
        this.pose = pose;
        this.width = width;
        this.length = length;
        this.axes = new Axis[] {new Axis(1, 0), new Axis(1, 0)};
        this.vertices = new Translation2d[5];
    }

    @Override
    public Axis[] getAxes() {
        axes[0].setFromRotation(pose.getRotation());
        axes[1].setFromRotation(pose.getRotation().plus(Rotation2d.kCW_90deg));
        return axes;
    }

    @Override
    public Interval project(Axis axis) {
        updateVertices();
        return axis.project(vertices);
    }

    private void updateVertices() {
        vertices[0] = pose.getTranslation()
            .plus(new Translation2d(length / 2.0, width / 2.0).rotateBy(pose.getRotation()));
        vertices[1] = pose.getTranslation()
            .plus(new Translation2d(-length / 2.0, width / 2.0).rotateBy(pose.getRotation()));
        vertices[2] = pose.getTranslation()
            .plus(new Translation2d(-length / 2.0, -width / 2.0).rotateBy(pose.getRotation()));
        vertices[3] = pose.getTranslation()
            .plus(new Translation2d(length / 2.0, -width / 2.0).rotateBy(pose.getRotation()));
        vertices[4] = vertices[0];
    }

    @Override
    public Translation2d getCenter() {
        return pose.getTranslation();
    }

    /** Override rectangle pose. */
    public void setPose(Pose2d pose) {
        this.pose = pose;
    }

    @Override
    public void drawImpl() {
        updateVertices();
        Logger.recordOutput(name, vertices);
    }

}
