package frc.lib.mut;

import static edu.wpi.first.units.Units.Radians;
import java.util.Objects;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;

/**
 * Mutable version of {@link Rotation2d}
 *
 * A rotation in a 2D coordinate frame represented by a point on the unit circle (cosine and sine).
 *
 * <p>
 * The angle is continuous, that is if a Rotation2d is constructed with 361 degrees, it will return
 * 361 degrees. This allows algorithms that wouldn't want to see a discontinuity in the rotations as
 * it sweeps past from 360 to 0 on the second time around.
 */
public class MutableRotation2d {

    private double value;
    private double cos;
    private double sin;

    /** Constructs a Rotation2d with a default angle of 0 degrees. */
    public MutableRotation2d() {
        this.value = 0.0;
        this.cos = 1.0;
        this.sin = 0.0;
    }

    /** Construct from immutable */
    public MutableRotation2d(Rotation2d imm) {
        this.value = imm.getRadians();
        this.cos = imm.getCos();
        this.sin = imm.getSin();
    }

    /** Create immutable from this. */
    public Rotation2d toImmutable() {
        return new Rotation2d(value);
    }

    /**
     * Constructs a Rotation2d with the given radian value.
     *
     * @param value The value of the angle in radians.
     */
    public void setRadians(double value) {
        this.value = value;
        this.cos = Math.cos(value);
        this.sin = Math.sin(value);
    }

    /**
     * Constructs a Rotation2d with the given x and y (cosine and sine) components.
     *
     * @param x The x component or cosine of the rotation.
     * @param y The y component or sine of the rotation.
     */
    public void setXY(double x, double y) {
        double magnitude = Math.hypot(x, y);
        if (magnitude > 1e-6) {
            this.cos = x / magnitude;
            this.sin = y / magnitude;
        } else {
            this.cos = 1.0;
            this.sin = 0.0;
            MathSharedStore.reportError("x and y components of Rotation2d are zero\n",
                Thread.currentThread().getStackTrace());
        }
        value = Math.atan2(this.sin, this.cos);
    }

    /**
     * Constructs a Rotation2d with the given angle.
     *
     * @param angle The angle of the rotation.
     */
    public void setAngle(Angle angle) {
        this.setRadians(angle.in(Radians));
    }

    /**
     * Constructs and returns a Rotation2d with the given degree value.
     *
     * @param degrees The value of the angle in degrees.
     */
    public void setDegrees(double degrees) {
        this.setRadians(Units.degreesToRadians(degrees));
    }

    /**
     * Constructs and returns a Rotation2d with the given number of rotations.
     *
     * @param rotations The value of the angle in rotations.
     * @see edu.wpi.first.math.MathUtil#angleModulus(double) to constrain the angle within (-π, π]
     */
    public void setRotations(double rotations) {
        this.setRadians(Units.rotationsToRadians(rotations));
    }

    /**
     * Adds two rotations together, with the result being bounded between -π and π.
     *
     * <p>
     * For example, <code>Rotation2d.fromDegrees(30).plus(Rotation2d.fromDegrees(60))</code> equals
     * <code>Rotation2d(Math.PI/2.0)</code>
     *
     * @param other The rotation to add.
     * @param out out parameter
     * @return The sum of the two rotations.
     */
    public MutableRotation2d plus(MutableRotation2d other, MutableRotation2d out) {
        return rotateBy(other, out);
    }

    /**
     * Adds two rotations together, with the result being bounded between -π and π.
     *
     * <p>
     * For example, <code>Rotation2d.fromDegrees(30).plus(Rotation2d.fromDegrees(60))</code> equals
     * <code>Rotation2d(Math.PI/2.0)</code>
     *
     * @param other The rotation to add.
     * @param out out parameter
     * @return The sum of the two rotations.
     */
    public MutableRotation2d plus(Rotation2d other, MutableRotation2d out) {
        return rotateBy(other, out);
    }

    /**
     * Adds two rotations together, with the result being bounded between -π and π.
     *
     * <p>
     * For example, <code>Rotation2d.fromDegrees(30).plus(Rotation2d.fromDegrees(60))</code> equals
     * <code>Rotation2d(Math.PI/2.0)</code>
     *
     * @param other The rotation to add.
     * @return The sum of the two rotations.
     */
    public MutableRotation2d plus(MutableRotation2d other) {
        return plus(other, new MutableRotation2d());
    }

    /**
     * Adds two rotations together, with the result being bounded between -π and π.
     *
     * <p>
     * For example, <code>Rotation2d.fromDegrees(30).plus(Rotation2d.fromDegrees(60))</code> equals
     * <code>Rotation2d(Math.PI/2.0)</code>
     *
     * @param other The rotation to add.
     * @return The sum of the two rotations.
     */
    public MutableRotation2d plus(Rotation2d other) {
        return plus(other, new MutableRotation2d());
    }

    /**
     * Subtracts the new rotation from the current rotation and returns the new rotation.
     *
     * <p>
     * For example, <code>Rotation2d.fromDegrees(10).minus(Rotation2d.fromDegrees(100))</code>
     * equals <code>Rotation2d(-Math.PI/2.0)</code>
     *
     * @param other The rotation to subtract.
     * @param out out parameter
     * @return The difference between the two rotations.
     */
    public MutableRotation2d minus(MutableRotation2d other, MutableRotation2d out) {
        return rotateBy(-other.value, out);
    }

    /**
     * Takes the inverse of the current rotation. This is simply the negative of the current angular
     * value.
     *
     * @param out out parameter
     * @return The inverse of the current rotation.
     */
    public MutableRotation2d unaryMinus(MutableRotation2d out) {
        out.setRadians(-value);
        return out;
    }

    /**
     * Multiplies the current rotation by a scalar.
     *
     * @param scalar The scalar.
     * @param out out parameter
     * @return The new scaled Rotation2d.
     */
    public MutableRotation2d times(double scalar, MutableRotation2d out) {
        out.setRadians(value * scalar);
        return out;
    }

    /**
     * Divides the current rotation by a scalar.
     *
     * @param scalar The scalar.
     * @param out out parameter
     * @return The new scaled Rotation2d.
     */
    public MutableRotation2d div(double scalar, MutableRotation2d out) {
        return times(1.0 / scalar, out);
    }

    /**
     * Adds the new rotation to the current rotation using a rotation matrix.
     *
     * <p>
     * The matrix multiplication is as follows:
     *
     * <pre>
     * [cos_new]   [other.cos, -other.sin][cos]
     * [sin_new] = [other.sin,  other.cos][sin]
     * value_new = atan2(sin_new, cos_new)
     * </pre>
     *
     * @param other The rotation to rotate by.
     * @param out out parameter
     * @return The new rotated Rotation2d.
     */
    public MutableRotation2d rotateBy(MutableRotation2d other, MutableRotation2d out) {
        out.setXY(cos * other.cos - sin * other.sin, cos * other.sin + sin * other.cos);
        return out;
    }

    /**
     * Adds the new rotation to the current rotation using a rotation matrix.
     *
     * <p>
     * The matrix multiplication is as follows:
     *
     * <pre>
     * [cos_new]   [other.cos, -other.sin][cos]
     * [sin_new] = [other.sin,  other.cos][sin]
     * value_new = atan2(sin_new, cos_new)
     * </pre>
     *
     * @param other The rotation to rotate by.
     * @param out out parameter
     * @return The new rotated Rotation2d.
     */
    public MutableRotation2d rotateBy(Rotation2d other, MutableRotation2d out) {
        out.setXY(cos * other.getCos() - sin * other.getSin(),
            cos * other.getSin() + sin * other.getCos());
        return out;
    }

    private MutableRotation2d rotateBy(double other, MutableRotation2d out) {
        double other_cos = Math.cos(other);
        double other_sin = Math.sin(other);
        out.setXY(cos * other_cos - sin * other_sin, cos * other_sin + sin * other_cos);
        return out;
    }

    /**
     * Returns the radian value of the Rotation2d.
     *
     * @return The radian value of the Rotation2d.
     */
    public double getRadians() {
        return value;
    }

    /**
     * Returns the degree value of the Rotation2d.
     *
     * @return The degree value of the Rotation2d.
     * @see edu.wpi.first.math.MathUtil#inputModulus(double, double, double) to constrain the angle
     *      within (-180, 180]
     */
    public double getDegrees() {
        return Math.toDegrees(value);
    }

    /**
     * Returns the number of rotations of the Rotation2d.
     *
     * @return The number of rotations of the Rotation2d.
     */
    public double getRotations() {
        return Units.radiansToRotations(value);
    }

    /**
     * Returns the cosine of the Rotation2d.
     *
     * @return The cosine of the Rotation2d.
     */
    public double getCos() {
        return cos;
    }

    /**
     * Returns the sine of the Rotation2d.
     *
     * @return The sine of the Rotation2d.
     */
    public double getSin() {
        return sin;
    }

    /**
     * Returns the tangent of the Rotation2d.
     *
     * @return The tangent of the Rotation2d.
     */
    public double getTan() {
        return sin / cos;
    }

    @Override
    public String toString() {
        return String.format("Rotation2d(Rads: %.2f, Deg: %.2f)", value, Math.toDegrees(value));
    }

    /**
     * Checks equality between this Rotation2d and another object.
     *
     * @param obj The other object.
     * @return Whether the two objects are equal or not.
     */
    @Override
    public boolean equals(Object obj) {
        return (obj instanceof Rotation2d other
            && Math.hypot(cos - other.getCos(), sin - other.getSin()) < 1E-9)
            || (obj instanceof MutableRotation2d other2
                && Math.hypot(cos - other2.getCos(), sin - other2.getSin()) < 1E-9);
    }

    @Override
    public int hashCode() {
        return Objects.hash(value);
    }

}
