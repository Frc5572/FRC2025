package frc.lib.math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Axis {

    private double xDir;
    private double yDir;


    public Axis(double xDir, double yDir) {
        setDirectionImpl(xDir, yDir);
    }

    public static Axis fromRotation(Rotation2d rot) {
        return new Axis(rot.getCos(), rot.getSin());
    }

    public void setFromRotation(Rotation2d rot) {
        setDirectionImpl(rot.getCos(), rot.getSin());
    }

    public Axis unaryMinus() {
        return new Axis(-xDir, -yDir);
    }

    public double getX() {
        return xDir;
    }

    public double getY() {
        return yDir;
    }

    public void setDirection(double xDir, double yDir) {
        setDirectionImpl(xDir, yDir);
    }

    private void setDirectionImpl(double xDir, double yDir) {
        double norm = Math.hypot(xDir, yDir);
        this.xDir = xDir / norm;
        this.yDir = yDir / norm;
    }

    public double dot(Translation2d point) {
        return this.xDir * point.getX() + this.yDir * point.getY();
    }

    public Interval project(Translation2d[] points) {
        double v = 0.0;
        Translation2d p = points[0];
        double min = this.dot(p);
        double max = min;

        for (int i = 1; i < points.length; i++) {
            p = points[i];
            v = this.dot(p);

            if (v < min) {
                min = v;
            } else if (v > max) {
                max = v;
            }
        }

        return new Interval(min, max);
    }

}
