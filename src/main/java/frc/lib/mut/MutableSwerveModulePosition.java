package frc.lib.mut;

import edu.wpi.first.math.kinematics.SwerveModulePosition;

/** Mutable version of {@link SwerveModulePosition} */
public class MutableSwerveModulePosition {

    /** Distance measured by the wheel of the module. */
    public double distanceMeters;

    /** Angle of the module. */
    public MutableRotation2d angle;

    public MutableSwerveModulePosition(double distanceMeters, MutableRotation2d angle) {
        this.distanceMeters = distanceMeters;
        this.angle = angle;
    }

    public MutableSwerveModulePosition() {
        this(0.0, new MutableRotation2d());
    }

    public SwerveModulePosition toImmutable() {
        return new SwerveModulePosition(distanceMeters, angle.toImmutable());
    }

}
