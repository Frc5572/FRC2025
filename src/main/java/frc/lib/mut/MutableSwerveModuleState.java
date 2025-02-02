package frc.lib.mut;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Mutable version of {@link SwerveModuleState} */
public class MutableSwerveModuleState {

    /** Speed of the wheel of the module. */
    public double speedMetersPerSecond;

    /** Angle of the module. */
    public MutableRotation2d angle;

    public MutableSwerveModuleState(double speedMetersPerSecond, MutableRotation2d angle) {
        this.speedMetersPerSecond = speedMetersPerSecond;
        this.angle = angle;
    }

    public MutableSwerveModuleState() {
        this(0.0, new MutableRotation2d());
    }

    public SwerveModuleState toImmutable() {
        return new SwerveModuleState(speedMetersPerSecond, angle.toImmutable());
    }

    public MutableSwerveModuleState fromImmutable(SwerveModuleState state) {
        this.speedMetersPerSecond = state.speedMetersPerSecond;
        this.angle.setRadians(state.angle.getRadians());
        return this;
    }

    private final MutableRotation2d tempRotation = new MutableRotation2d();

    /**
     * Minimize the change in heading this swerve module state would require by potentially
     * reversing the direction the wheel spins. If this is used with the PIDController class's
     * continuous input functionality, the furthest a wheel will ever rotate is 90 degrees.
     *
     * @param currentAngle The current module angle.
     */
    public void optimize(MutableRotation2d currentAngle) {
        var delta = angle.minus(currentAngle, tempRotation);
        if (Math.abs(delta.getDegrees()) > 90.0) {
            speedMetersPerSecond *= -1;
            angle.rotateBy(Rotation2d.kPi, angle);
        }
    }

    /**
     * Scales speed by cosine of angle error. This scales down movement perpendicular to the desired
     * direction of travel that can occur when modules change directions. This results in smoother
     * driving.
     *
     * @param currentAngle The current module angle.
     */
    public void cosineScale(MutableRotation2d currentAngle) {
        speedMetersPerSecond *= angle.minus(currentAngle, tempRotation).getCos();
    }

}
