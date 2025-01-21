package frc.lib.math;

import static edu.wpi.first.units.Units.Meters;
import static java.lang.Math.acos;
import static java.lang.Math.sqrt;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import frc.lib.draw.DrawingUtils.Drawable;

public class Circle implements Drawable {

    public Translation2d center;
    public Distance radius;
    public int resoultion;
    public String color;

    private final String key;

    public Circle(String key, Translation2d center, Distance radius, String color, int resoultion) {
        this.key = key;
        this.center = center;
        this.radius = radius;
        this.resoultion = resoultion;
        this.color = color;
    }

    public Circle(String key, Translation2d center, Distance radius, String color) {
        this(key, center, radius, color, 20);
    }

    @Override
    public void draw() {
        Translation2d[] res = new Translation2d[resoultion + 1];
        for (int i = 0; i < resoultion + 1; i++) {
            double theta = (double) i / (double) resoultion * Math.PI * 2.0;
            double x = this.center.getX() + Math.cos(theta) * this.radius.in(Meters);
            double y = this.center.getY() + Math.sin(theta) * this.radius.in(Meters);
            res[i] = new Translation2d(x, y);
        }

        Logger.recordOutput(this.key, res);
    }

    @Override
    public ObjectNode layout(ObjectMapper om) {
        ObjectNode source = om.createObjectNode();
        source.put("type", "trajectory");
        source.put("logKey", "NT:/AdvantageKit/RealOutputs/" + this.key);
        source.put("logType", "Translation2d[]");
        source.put("visible", true);
        ObjectNode options = om.createObjectNode();
        options.put("color", color);
        options.put("size", "normal");
        source.set("options", options);
        return source;
    }

    /**
     * https://stackoverflow.com/a/1084899
     */
    public boolean intersectsLine(Translation2d e, Translation2d l) {
        var d = l.minus(e);
        var f = e.minus(center);
        var r = radius.in(Meters);
        var a = dot(d, d);
        var b = dot(f.times(2), d);
        var c = dot(f, f) - r * r;
        var discriminant = b * b - 4 * a * c;
        if (discriminant < 0) {
            // no intersection
            return false;
        } else {
            // ray didn't totally miss sphere,
            // so there is a solution to
            // the equation.

            discriminant = sqrt(discriminant);

            // either solution may be on or off the ray so need to test both
            // t1 is always the smaller value, because BOTH discriminant and
            // a are nonnegative.
            double t1 = (-b - discriminant) / (2 * a);
            double t2 = (-b + discriminant) / (2 * a);

            // 3x HIT cases:
            // -o-> --|--> | | --|->
            // Impale(t1 hit,t2 hit), Poke(t1 hit,t2>1), ExitWound(t1<0, t2 hit),

            // 3x MISS cases:
            // -> o o -> | -> |
            // FallShort (t1>1,t2>1), Past (t1<0,t2<0), CompletelyInside(t1<0, t2>1)
            if (t1 >= 0 && t1 <= 1) {
                // t1 is the intersection, and it's closer than t2
                // (since t1 uses -b - discriminant)
                // Impale, Poke
                return true;
            }
            // here t1 didn't intersect so we are either started
            // inside the sphere or completely past it
            if (t2 >= 0 && t2 <= 1) {
                // ExitWound
                return true;
            }

            // no intn: FallShort, Past, CompletelyInside
            return false;
        }
    }

    private static double dot(Translation2d a, Translation2d b) {
        return a.getX() * b.getX() + a.getY() * b.getY();
    }

    public Optional<Pair<Rotation2d, Rotation2d>> circleTangentAngles(Translation2d p) {
        Translation2d diff = p.minus(center);
        double d = diff.getNorm();
        double det = radius.in(Meters) / d;
        if (det > 1.0 || det < -1.0) {
            return Optional.empty();
        }
        double dtheta = acos(det);
        if (dtheta < 0.0) {
            dtheta = -dtheta;
        }

        Rotation2d dAngle = Rotation2d.fromRadians(dtheta);
        return Optional.of(Pair.of(diff.getAngle().plus(dAngle), diff.getAngle().minus(dAngle)));
    }

    public Optional<Pair<Pair<Rotation2d, Translation2d>, Pair<Rotation2d, Translation2d>>> circleTangentPoints(
        Translation2d p) {
        return circleTangentAngles(p).map(x -> {
            return Pair.of(
                Pair.of(x.getFirst(),
                    center.plus(new Translation2d(radius.in(Meters), x.getFirst()))),
                Pair.of(x.getSecond(),
                    center.plus(new Translation2d(radius.in(Meters), x.getSecond()))));
        });
    }

}
