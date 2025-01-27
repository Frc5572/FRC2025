package frc.lib.math;

import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.util.viz.Drawable;

public class SweptRectangles implements ConvexShape, Drawable {

    private final String name;
    private final Rectangle rectA;
    private final Rectangle rectB;

    public SweptRectangles(String name, Rectangle rectA, Rectangle rectB) {
        this.name = name;
        this.rectA = rectA;
        this.rectB = rectB;
    }

    @Override
    public void draw() {
        Translation2d[] hull = convexHull();
        Translation2d[] draw = new Translation2d[hull.length + 1];
        System.arraycopy(hull, 0, draw, 0, hull.length);
        draw[hull.length] = hull[0];
        Logger.recordOutput(name, draw);
    }

    private final Axis[] axes = new Axis[5];

    @Override
    public Axis[] getAxes() {
        Translation2d diff = rectA.getCenter().minus(rectB.getCenter());
        System.arraycopy(rectA.getAxes(), 0, axes, 0, 2);
        System.arraycopy(rectB.getAxes(), 0, axes, 2, 2);
        axes[4] = new Axis(diff.getY(), -diff.getX());
        return axes;
    }

    @Override
    public Interval project(Axis axis) {
        return axis.project(convexHull());
    }

    @Override
    public Translation2d getCenter() {
        return rectA.getCenter().plus(rectB.getCenter()).div(2);
    }

    private Translation2d[] convexHull() {
        Translation2d[] pointsRaw = new Translation2d[8];
        System.arraycopy(rectA.getVertices(), 0, pointsRaw, 0, 4);
        System.arraycopy(rectB.getVertices(), 0, pointsRaw, 4, 4);

        ArrayList<Translation2d> hull = new ArrayList<>(6);

        int l = 0;
        for (int i = 1; i < 8; i++) {
            if (pointsRaw[i].getX() < pointsRaw[l].getX()) {
                l = i;
            }
        }

        int p = l, q;
        do {
            hull.add(pointsRaw[p]);
            q = (p + 1) % pointsRaw.length;
            for (int i = 0; i < pointsRaw.length; i++) {
                if (isCCW(pointsRaw[p], pointsRaw[i], pointsRaw[q])) {
                    q = i;
                }
            }

            p = q;
        } while (p != l);

        return hull.toArray(Translation2d[]::new);
    }

    private static boolean isCCW(Translation2d p, Translation2d c, Translation2d n) {
        double val = ((c.getY() - p.getY()) * (n.getX() - c.getX()))
            - ((c.getX() - p.getX()) * (n.getY() - c.getY()));
        return val < 0;
    }

}
