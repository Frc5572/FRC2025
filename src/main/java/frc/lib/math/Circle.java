package frc.lib.math;

import static java.lang.Math.acos;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.util.viz.Drawable;

public class Circle implements ConvexShape, Drawable {

    private final String name;
    private Translation2d center;
    private double radius;

    public Circle(String name, Translation2d center, double radius) {
        this.name = name;
        this.center = center;
        this.radius = radius;
    }

    private static final int RESOLUTION = 20;

    @Override
    public void drawImpl() {
        Translation2d[] points = new Translation2d[RESOLUTION + 1];
        for (int i = 0; i < RESOLUTION + 1; i++) {
            points[i] = center.plus(new Translation2d(radius,
                Rotation2d.fromRotations((double) i / (double) RESOLUTION)));
        }
        Logger.recordOutput(name, points);
    }

    private static final Axis[] axes = new Axis[0];

    @Override
    public Axis[] getAxes() {
        return axes;
    }

    @Override
    public Interval project(Axis axis) {
        double c = axis.dot(center);
        return new Interval(c - this.radius, c + this.radius);
    }

    @Override
    public Translation2d getCenter() {
        return center;
    }

    public void setCenter(Translation2d center) {
        this.center = center;
    }

    public double getRadius() {
        return radius;
    }

    public void setRadius(double radius) {
        this.radius = radius;
    }

    public boolean contains(Translation2d pt) {
        return pt.minus(center).getNorm() < radius;
    }

    public double sdf(Translation2d pt) {
        return pt.minus(center).getNorm() - radius;
    }

    public Rotation2d getAngle(Translation2d pt) {
        return pt.minus(center).getAngle();
    }

    public Translation2d getVertex(Rotation2d rotation) {
        return getVertex(rotation, 0);
    }

    public Translation2d getVertex(Rotation2d rotation, double extra) {
        return center.plus(new Translation2d(radius + extra, rotation));
    }

    /** Get angles for points that are tangent to this and colinear with a given point. */
    public RotationInterval circleTangentAngles(Translation2d p, Rotation2d insideAngle) {
        Translation2d diff = p.minus(center);
        double d = diff.getNorm();
        double det = radius / d;
        if (det > 1.0 || det < -1.0) {
            var angle = getAngle(p);
            return new RotationInterval(angle.minus(insideAngle), angle.plus(insideAngle));
        }
        double dtheta = acos(det);
        if (dtheta < 0.0) {
            dtheta = -dtheta;
        }

        Rotation2d dAngle = Rotation2d.fromRadians(dtheta);
        return RotationInterval.acute(diff.getAngle().plus(dAngle), diff.getAngle().minus(dAngle));
    }

    public RotationInterval circleTangentAngles(Translation2d p) {
        return circleTangentAngles(p, Rotation2d.fromDegrees(5));
    }

}
