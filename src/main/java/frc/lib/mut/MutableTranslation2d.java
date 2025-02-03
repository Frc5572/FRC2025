package frc.lib.mut;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Mutable version of {@link Translation2d} */
public class MutableTranslation2d {

    private double x;
    private double y;

    /** Mutable version of {@link Translation2d} */
    public MutableTranslation2d() {
        this(0.0, 0.0);
    }

    /** Mutable version of {@link Translation2d} */
    public MutableTranslation2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /** Mutable version of {@link Translation2d} */
    public MutableTranslation2d(Translation2d other) {
        this.x = other.getX();
        this.y = other.getY();
    }

    /** Convert to immutable version */
    public Translation2d toImmutable() {
        return new Translation2d(x, y);
    }

    /** Get X value */
    public double getX() {
        return x;
    }

    /** Set X value */
    public void setX(double x) {
        this.x = x;
    }

    /** Get Y value */
    public double getY() {
        return y;
    }

    /** Set Y value */
    public void setY(double y) {
        this.y = y;
    }

    /** Set X and Y values simultaneously and return self. */
    public MutableTranslation2d setXY(double x, double y) {
        this.x = x;
        this.y = y;
        return this;
    }

    /** Rotate about the origin with a given rotation */
    public MutableTranslation2d rotateBy(MutableRotation2d rotation, MutableTranslation2d out) {
        double x = this.x * rotation.getCos() - this.y * rotation.getSin();
        double y = this.x * rotation.getSin() + this.y * rotation.getCos();
        return out.setXY(x, y);
    }

    /** Set values based on rotation and distance */
    public MutableTranslation2d setDistanceAngle(double distance, MutableRotation2d angle) {
        return this.setXY(distance * angle.getCos(), distance * angle.getSin());
    }

    /** Set values based on rotation and distance */
    public MutableTranslation2d setDistanceAngle(double distance, Rotation2d angle) {
        return this.setXY(distance * angle.getCos(), distance * angle.getSin());
    }

    /** Set values based on rotation and distance */
    public MutableTranslation2d setDistanceAngle(double distance, double radians) {
        return this.setXY(distance * Math.cos(radians), distance * Math.sin(radians));
    }

    /** Get distance from origin */
    public double getNorm() {
        return Math.hypot(x, y);
    }

    /** Returns the angle this translation forms with the positive X axis. */
    public MutableRotation2d getAngle(MutableRotation2d out) {
        out.setXY(x, y);
        return out;
    }

    /** Returns the sum of two translations in 2D space. */
    public MutableTranslation2d plus(MutableTranslation2d other, MutableTranslation2d out) {
        out.x = this.x + other.x;
        out.y = this.y + other.y;
        return out;
    }

    /** Returns the difference between two translations. */
    public MutableTranslation2d minus(MutableTranslation2d other, MutableTranslation2d out) {
        out.x = this.x - other.x;
        out.y = this.y - other.y;
        return out;
    }

    /**
     * Returns the inverse of the current translation. This is equivalent to rotating by 180
     * degrees, flipping the point over both axes, or negating all components of the translation.
     */
    public MutableTranslation2d unaryMinus(MutableTranslation2d out) {
        out.x = -this.x;
        out.y = -this.y;
        return out;
    }

    /** Returns the translation multiplied by a scalar. */
    public MutableTranslation2d times(double scalar, MutableTranslation2d out) {
        out.x = this.x * scalar;
        out.y = this.y * scalar;
        return out;
    }

    /** Returns the translation divided by a scalar. */
    public MutableTranslation2d div(double scalar, MutableTranslation2d out) {
        return this.times(1.0 / scalar, out);
    }

}
