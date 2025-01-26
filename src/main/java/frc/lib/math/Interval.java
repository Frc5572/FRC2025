package frc.lib.math;

public class Interval {

    private double min;
    private double max;

    public Interval(double min, double max) {
        this.min = min;
        this.max = max;
    }

    public double getMin() {
        return min;
    }

    public void setMin(double min) {
        this.min = min;
    }

    public double getMax() {
        return max;
    }

    public void setMax(double max) {
        this.max = max;
    }



    public boolean overlaps(Interval other) {
        return !(this.min > other.max || other.min > this.max);
    }

    public double getOverlap(Interval other) {
        // make sure they overlap
        if (this.overlaps(other)) {
            return Math.min(this.max, other.max) - Math.max(this.min, other.min);
        }
        return 0;
    }

    public boolean contains(Interval other) {
        return other.min > this.min && other.max < this.max;
    }

}
