package frc.lib.math;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.util.viz.Drawable;

public class LineSegment implements ConvexShape, Drawable {

    private Translation2d from;
    private Translation2d to;
    private final String name;

    public LineSegment(String name, Translation2d from, Translation2d to) {
        this.from = from;
        this.to = to;
        this.name = name;
    }

    @Override
    public void drawImpl() {
        Logger.recordOutput(name, new Translation2d[] {from, to});
    }

    private final Axis[] axes = new Axis[1];

    @Override
    public Axis[] getAxes() {
        axes[0].setDirection(to.getY() - from.getY(), from.getX() - to.getX());
        return axes;
    }

    @Override
    public Interval project(Axis axis) {
        return axis.project(new Translation2d[] {from, to});
    }

    @Override
    public Translation2d getCenter() {
        return from.plus(to).div(2);
    }

    public Translation2d getFrom() {
        return from;
    }

    public void setFrom(Translation2d from) {
        this.from = from;
    }

    public Translation2d getTo() {
        return to;
    }

    public void setTo(Translation2d to) {
        this.to = to;
    }



}
