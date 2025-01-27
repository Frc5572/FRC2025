package frc.lib.math;

import static java.lang.Math.acos;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.util.Tuples.Tuple2;
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
    public void draw() {
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

    /** Get angles for points that are tangent to this and colinear with a given point. */
    public Optional<Tuple2<Rotation2d, Rotation2d>> circleTangentAngles(Translation2d p) {
        Translation2d diff = p.minus(center);
        double d = diff.getNorm();
        double det = radius / d;
        if (det > 1.0 || det < -1.0) {
            return Optional.empty();
        }
        double dtheta = acos(det);
        if (dtheta < 0.0) {
            dtheta = -dtheta;
        }

        Rotation2d dAngle = Rotation2d.fromRadians(dtheta);
        return Optional
            .of(new Tuple2<>(diff.getAngle().plus(dAngle), diff.getAngle().minus(dAngle)));
    }

    /** Same as {@link circleTangentAngles} but additionally gets the points. */
    public Optional<Tuple2<Tuple2<Rotation2d, Translation2d>, Tuple2<Rotation2d, Translation2d>>> circleTangentPoints(
        Translation2d p) {
        return circleTangentAngles(p).map(x -> {
            return new Tuple2<>(
                new Tuple2<>(x._0(), center.plus(new Translation2d(radius, x._0()))),
                new Tuple2<>(x._1(), center.plus(new Translation2d(radius, x._1()))));
        });
    }

}
