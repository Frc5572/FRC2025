package frc.lib.math;

import org.littletonrobotics.junction.Logger;
import frc.lib.util.viz.Drawable;

public class Penetration implements Drawable {

    private final String name;

    public Penetration(String name) {
        this.name = name;
    }

    private double normalX = 0.0;
    private double normalY = 0.0;
    private double depth = 0.0;

    public void setNormal(double x, double y) {
        this.normalX = x;
        this.normalY = y;
    }

    public void setDepth(double depth) {
        this.depth = depth;
    }

    public double getNormalX() {
        return normalX;
    }

    public double getNormalY() {
        return normalY;
    }

    public double getDepth() {
        return depth;
    }

    @Override
    public void draw() {
        Logger.recordOutput(name + "/NormalX", normalX);
        Logger.recordOutput(name + "/NormalY", normalY);
        Logger.recordOutput(name + "/Depth", depth);
    }



}
