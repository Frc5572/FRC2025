package frc.lib.math;

import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

public class Circle {

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

        arr[0] = 1;
        arr[1] = -m1;
        arr[2] = xint1;
        arr[3] = 1;
        arr[4] = -m2;
        arr[5] = xint2;

        return true;
    }

}
