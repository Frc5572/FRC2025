package frc.lib.math;

import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

public class Circle {

    /**
     * https://mathworld.wolfram.com/Circle-LineIntersection.html
     */
    public static boolean intersectsCircle(double x1a, double y1a, double x2a, double y2a,
        double cx, double cy, double r) {
        double x1 = (x1a - cx);
        double y1 = (y1a - cy);
        double x2 = (x2a - cx);
        double y2 = (y2a - cy);

        double dx = x2 - x1;
        double dy = y2 - y1;

        double dr = sqrt(dx * dx + dy * dy);
        double D = x1 * y2 - x2 * y1;

        return r * r * dr * dr - D * D >= 0;
    }

    /**
     * Calculate tangent lines given a circle and point both tangent lines must intersect at.
     * 
     * @param arr a double array of length 6. Modified with the result. arr[0..3] are `A`, `B`, and
     *        `C` in the standard line equation `Ax + By = C`. arr[3..6] is the same for the second
     *        line.
     */
    public static boolean calculateCircleTangents(double p_x, double p_y, double c_x, double c_y,
        double r, double[] arr) {
        if (calculateCircleTangentsX(p_x, p_y, c_x, c_y, r, arr)) {
            return true;
        } else if (calculateCircleTangentsY(p_x, p_y, c_x, c_y, r, arr)) {
            return true;
        } else {
            return false;
        }
    }

    private static boolean calculateCircleTangentsX(double p_x, double p_y, double c_x, double c_y,
        double r, double[] arr) {
        double d1 = -pow(c_x, 2) + 2 * c_x * p_x - pow(p_x, 2) + pow(r, 2);
        double d2 = (-c_x + p_x + r) * (c_x - p_x + r);

        if (d1 == 0 || d2 == 0) {
            return false;
        }

        double pn = r * sqrt(pow(c_x, 2) - 2 * c_x * p_x + pow(c_y, 2) - 2 * c_y * p_y + pow(p_x, 2)
            + pow(p_y, 2) - pow(r, 2)) / d1;
        double c = (c_x - p_x) * (c_y - p_y) / d2;

        double m1 = pn - c;
        double m2 = -pn - c;
        double yint1 = p_y - p_x * m1;
        double yint2 = p_y - p_x * m2;

        if (m1 > m2) {
            double m3 = m2;
            m2 = m1;
            m1 = m3;
            double yint3 = yint2;
            yint2 = yint1;
            yint1 = yint3;
        }

        arr[0] = -m1;
        arr[1] = 1;
        arr[2] = yint1;
        arr[3] = -m2;
        arr[4] = 1;
        arr[5] = yint2;

        return true;
    }

    private static boolean calculateCircleTangentsY(double p_x, double p_y, double c_x, double c_y,
        double r, double[] arr) {
        double d1 = -pow(c_y, 2) + 2 * c_y * p_y - pow(p_y, 2) + pow(r, 2);
        double d2 = (-c_y + p_y + r) * (c_y - p_y + r);

        if (d1 == 0 || d2 == 0) {
            return false;
        }

        double pn = r * sqrt(pow(c_x, 2) - 2 * c_x * p_x + pow(c_y, 2) - 2 * c_y * p_y + pow(p_x, 2)
            + pow(p_y, 2) - pow(r, 2)) / d1;
        double c = (c_x - p_x) * (c_y - p_y) / d2;

        double m1 = pn - c;
        double m2 = -pn - c;
        double xint1 = p_x - p_y * m1;
        double xint2 = p_x - p_y * m2;

        if (m1 > m2) {
            double m3 = m2;
            m2 = m1;
            m1 = m3;
            double xint3 = xint2;
            xint2 = xint1;
            xint1 = xint3;
        }

        arr[0] = 1;
        arr[1] = -m1;
        arr[2] = xint1;
        arr[3] = 1;
        arr[4] = -m2;
        arr[5] = xint2;

        return true;
    }

    public static boolean tangentIntersections(double[] from, double[] to, double[] out) {
        boolean success = false;
        if (!tangentIntersection(from[0], from[1], from[2], to[0], to[1], to[2], out, 0)) {
            out[0] = 0;
            out[1] = 0;
        } else {
            success = true;
        }
        if (!tangentIntersection(from[3], from[4], from[5], to[0], to[1], to[2], out, 2)) {
            out[2] = 0;
            out[3] = 0;
        } else {
            success = true;
        }
        if (!tangentIntersection(from[0], from[1], from[2], to[3], to[4], to[5], out, 4)) {
            out[4] = 0;
            out[5] = 0;
        } else {
            success = true;
        }
        if (!tangentIntersection(from[3], from[4], from[5], to[3], to[4], to[5], out, 6)) {
            out[6] = 0;
            out[7] = 0;
        } else {
            success = true;
        }
        return success;
    }

    private static boolean tangentIntersection(double a1, double b1, double c1, double a2,
        double b2, double c2, double[] out, int offset) {
        double det = a1 * b2 - a2 * b1;
        if (det == 0) {
            return false;
        }
        double x = (-b1 * c2 + b2 * c1) / det;
        double y = (a1 * c2 - a2 * c1) / det;

        out[offset] = x;
        out[offset + 1] = y;

        return true;
    }

}
