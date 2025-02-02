package frc.lib.mut;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Mutable version of {@link Translation2d} */
public class MutableTranslation2d {

    private double x;
    private double y;

    public MutableTranslation2d() {
        this(0.0, 0.0);
    }

    public MutableTranslation2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public MutableTranslation2d(Translation2d other) {
        this.x = other.getX();
        this.y = other.getY();
    }

    public Translation2d toImmutable() {
        return new Translation2d(x, y);
    }

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }

    public MutableTranslation2d setXY(double x, double y) {
        this.x = x;
        this.y = y;
        return this;
    }

    public MutableTranslation2d rotateBy(MutableRotation2d rotation, MutableTranslation2d out) {
        double x = this.x * rotation.getCos() - this.y * rotation.getSin();
        double y = this.x * rotation.getSin() + this.y * rotation.getCos();
        return out.setXY(x, y);
    }

    public MutableTranslation2d setDistanceAngle(double distance, MutableRotation2d angle) {
        return this.setXY(distance * angle.getCos(), distance * angle.getSin());
    }

    public MutableTranslation2d setDistanceAngle(double distance, Rotation2d angle) {
        return this.setXY(distance * angle.getCos(), distance * angle.getSin());
    }

    public MutableTranslation2d setDistanceAngle(double distance, double radians) {
        return this.setXY(distance * Math.cos(radians), distance * Math.sin(radians));
    }

    public double getNorm() {
        return Math.hypot(x, y);
    }

    public MutableRotation2d getAngle(MutableRotation2d out) {
        // TODO
        return out;
    }

    public MutableTranslation2d plus(MutableTranslation2d other, MutableTranslation2d out) {
        // TODO
        return out;
    }

    public MutableTranslation2d minus(MutableTranslation2d other, MutableTranslation2d out) {
        // TODO
        return out;
    }

    public MutableTranslation2d unaryMinus(MutableTranslation2d out) {
        // TODO
        return out;
    }

    public MutableTranslation2d times(double scalar, MutableTranslation2d out) {
        // TODO
        return out;
    }

    public MutableTranslation2d div(double scalar, MutableTranslation2d out) {
        // TODO
        return out;
    }

}
