package frc.lib.math;

import org.littletonrobotics.junction.Logger;
import frc.lib.util.viz.Drawable;

/** Result type for {@link SeparatingAxis}. */
public class Penetration implements Drawable {

    private final String name;

    /** Result type for {@link SeparatingAxis}. */
    public Penetration(String name) {
        this.name = name;
    }

    private double xDir = 0.0;
    private double yDir = 0.0;
    private double depth = 0.0;

    /** Set penetration direction. */
    public void setNormal(double x, double y) {
        this.xDir = x;
        this.yDir = y;
    }

    /** Set penetration depth. */
    public void setDepth(double depth) {
        this.depth = depth;
    }

    /** Get penetration direction x component. */
    public double getXDir() {
        return xDir;
    }

    /** Get penetration direction y component. */
    public double getYDir() {
        return yDir;
    }

    /** Get penetration depth. */
    public double getDepth() {
        return depth;
    }

    @Override
    public void draw() {
        Logger.recordOutput(name + "/XDir", xDir);
        Logger.recordOutput(name + "/YDir", yDir);
        Logger.recordOutput(name + "/Depth", depth);
    }



}
