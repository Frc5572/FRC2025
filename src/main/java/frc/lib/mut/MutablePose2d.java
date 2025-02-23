package frc.lib.mut;

import edu.wpi.first.math.geometry.Pose2d;

/** Mutable version of {@link Pose2d} */
public class MutablePose2d {

    private final MutableTranslation2d translation;
    private final MutableRotation2d rotation;

    public MutablePose2d() {
        this(new MutableTranslation2d(), new MutableRotation2d());
    }

    public MutablePose2d(MutableTranslation2d translation, MutableRotation2d rotation) {
        this.rotation = rotation;
        this.translation = translation;
    }

    public MutablePose2d(Pose2d pose) {
        this.rotation = new MutableRotation2d(pose.getRotation());
        this.translation = new MutableTranslation2d(pose.getTranslation());
    }

    public MutableTranslation2d getTranslation() {
        return translation;
    }

    public MutableRotation2d getRotation() {
        return rotation;
    }

}
