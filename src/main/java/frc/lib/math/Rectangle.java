package frc.lib.math;

import java.util.stream.Stream;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.mut.MutablePose2d;
import frc.lib.mut.MutableRotation2d;
import frc.lib.mut.MutableTranslation2d;
import frc.lib.util.viz.Drawable;

/** Rotating Rectangle Shape */
public class Rectangle implements ConvexShape, Drawable {

    public final double width;
    public final double length;

    private final Axis[] axes;
    private final MutableTranslation2d[] vertices;

    private MutablePose2d pose;

    private final String name;

    /** Rotating Rectangle Shape */
    public Rectangle(String name, Pose2d pose, double length, double width) {
        this.name = name;
        this.pose = new MutablePose2d(pose);
        this.width = width;
        this.length = length;
        this.axes = new Axis[] {new Axis(1, 0), new Axis(1, 0)};
        this.vertices = new MutableTranslation2d[5];
        for (int i = 0; i < 4; i++) {
            vertices[i] = new MutableTranslation2d();
        }
        this.cornerValues = new double[] {length / 2.0, width / 2.0, -length / 2.0, width / 2.0,
            -length / 2.0, -width / 2.0, length / 2.0, -width / 2.0};
    }

    private MutableRotation2d tempRotation = new MutableRotation2d();

    @Override
    public Axis[] getAxes() {
        axes[0].setFromRotation(pose.getRotation());
        axes[1].setFromRotation(pose.getRotation().plus(Rotation2d.kCW_90deg, tempRotation));
        return axes;
    }

    private final Interval tempInterval = new Interval(0, 0);

    @Override
    public Interval project(Axis axis) {
        updateVertices();
        return axis.project(vertices, tempInterval);
    }

    private final MutableTranslation2d[] corners =
        new MutableTranslation2d[] {new MutableTranslation2d(), new MutableTranslation2d(),
            new MutableTranslation2d(), new MutableTranslation2d()};

    private final double[] cornerValues;

    private void updateVertices() {
        for (int i = 0; i < 4; i++) {
            corners[i].setXY(cornerValues[2 * i], cornerValues[2 * i] + 1)
                .rotateBy(pose.getRotation(), corners[i]).plus(pose.getTranslation(), vertices[i]);
        }
        vertices[4] = vertices[0];
    }

    @Override
    public MutableTranslation2d getCenter() {
        return pose.getTranslation();
    }

    /** Override rectangle pose. */
    public void setPose(Pose2d pose) {
        this.setPose(pose.getX(), pose.getY(), pose.getRotation().getRadians());
    }

    public void setPose(double x, double y, double radians) {
        this.pose.getTranslation().setXY(x, y);
        this.pose.getRotation().setRadians(radians);
    }

    @Override
    public void drawImpl() {
        updateVertices();
        Logger.recordOutput(name,
            Stream.of(vertices).map((v) -> v.toImmutable()).toArray(Translation2d[]::new));
    }

}
