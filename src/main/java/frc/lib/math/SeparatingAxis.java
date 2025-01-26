package frc.lib.math;

import edu.wpi.first.math.geometry.Translation2d;

public class SeparatingAxis {

    public static boolean solve(ConvexShape shape1, ConvexShape shape2, Penetration penetration) {

        Axis n = null;
        double overlap = Double.MAX_VALUE;

        Axis[] axes1 = shape1.getAxes();
        Axis[] axes2 = shape2.getAxes();

        for (int i = 0; i < axes1.length; i++) {
            Axis axis = axes1[i];
            Interval intervalA = shape1.project(axis);
            Interval intervalB = shape2.project(axis);

            if (!intervalA.overlaps(intervalB)) {
                return false;
            } else {
                double o = intervalA.getOverlap(intervalB);
                // Check for containment
                if (intervalA.contains(intervalB) || intervalB.contains(intervalA)) {
                    // if containment exists then get the overlap plus the distance
                    // to between the two end points that are the closest
                    double max = Math.abs(intervalA.getMax() - intervalB.getMax());
                    double min = Math.abs(intervalA.getMin() - intervalB.getMin());
                    if (max > min) {
                        // if the min differences is less than the max then we need
                        // to flip the penetration axis
                        axis = axis.unaryMinus();
                        o += min;
                    } else {
                        o += max;
                    }
                }

                if (o < overlap) {
                    overlap = o;
                    n = axis;
                }
            }
        }

        for (int i = 0; i < axes2.length; i++) {
            Axis axis = axes2[i];
            Interval intervalA = shape1.project(axis);
            Interval intervalB = shape2.project(axis);

            if (!intervalA.overlaps(intervalB)) {
                return false;
            } else {
                double o = intervalA.getOverlap(intervalB);
                // Check for containment
                if (intervalA.contains(intervalB) || intervalB.contains(intervalA)) {
                    // if containment exists then get the overlap plus the distance
                    // to between the two end points that are the closest
                    double max = Math.abs(intervalA.getMax() - intervalB.getMax());
                    double min = Math.abs(intervalA.getMin() - intervalB.getMin());
                    if (max > min) {
                        // if the min differences is less than the max then we need
                        // to flip the penetration axis
                        axis = axis.unaryMinus();
                        o += min;
                    } else {
                        o += max;
                    }
                }

                if (o < overlap) {
                    overlap = o;
                    n = axis;
                }
            }
        }

        Translation2d c1 = shape1.getCenter();
        Translation2d c2 = shape2.getCenter();
        Translation2d cToc = c1.minus(c2);
        if (n.dot(cToc) < 0) {
            n = n.unaryMinus();
        }

        penetration.setNormal(n.getX(), n.getY());
        penetration.setDepth(overlap);

        return true;
    }
}
