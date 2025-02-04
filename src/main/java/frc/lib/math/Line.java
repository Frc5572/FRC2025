package frc.lib.math;

import static edu.wpi.first.units.Units.Meters;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.util.viz.Drawable;
import frc.robot.FieldConstants;

/** Represents a line in 2d space. Uses Ax + By + c = 0 representation. */
public class Line implements Drawable {

    public double a;
    public double b;
    public double c;
    public String name;

    /** Represents a line in 2d space. Uses Ax + By + c = 0 representation. */
    public Line(String name, double a, double b, double c) {
        this.name = name;
        this.a = a;
        this.b = b;
        this.c = c;
    }

    private static final double minX = -20.0;
    private static final double maxX = FieldConstants.fieldLength.in(Meters) + 20.0;
    private static final double minY = -20.0;
    private static final double maxY = FieldConstants.fieldWidth.in(Meters) + 20.0;

    @Override
    public void drawImpl() {
        if (a == 0 && b == 0) {
            throw new IllegalArgumentException("a and b cannot simultaneously be zero!");
        } else if (b == 0) {
            double x = -c / a;
            Logger.recordOutput(name,
                new Translation2d[] {new Translation2d(x, minY), new Translation2d(x, maxY)});
        } else if (a == 0) {
            double y = -c / b;
            Logger.recordOutput(name,
                new Translation2d[] {new Translation2d(minX, y), new Translation2d(maxX, y)});
        } else {
            double x1 = minX;
            double y1 = (-c - a * x1) / b;
            if (y1 > maxY) {
                y1 = maxY;
                x1 = (-c - b * y1) / a;
            } else if (y1 < minY) {
                y1 = minY;
                x1 = (-c - b * y1) / a;
            }
            double x2 = maxX;
            double y2 = (-c - a * x2) / b;
            if (y2 > maxY) {
                y2 = maxY;
                x2 = (-c - b * y2) / a;
            } else if (y2 < minY) {
                y2 = minY;
                x2 = (-c - b * y2) / a;
            }
            Logger.recordOutput(name,
                new Translation2d[] {new Translation2d(x1, y1), new Translation2d(x2, y2)});
        }
    }

    /** Get intersection of two lines. Returns null if lines are parallel. */
    public Translation2d intersection(Line other) {
        double d = a * other.b - b * other.a;
        if (d != 0) {
            double dx = -c * other.b + b * other.c;
            double dy = -a * other.c + c * other.a;
            return new Translation2d(dx / d, dy / d);
        } else {
            return null;
        }
    }

}
