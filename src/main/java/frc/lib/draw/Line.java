package frc.lib.draw;

import org.littletonrobotics.junction.Logger;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.draw.DrawingUtils.Drawable;
import frc.lib.math.FieldConstants;

public class Line implements Drawable {

    public Translation2d start;
    public Translation2d end;
    public String color;

    private final String key;

    public Line(String key, Translation2d start, Translation2d end, String color) {
        this.key = key;
        this.start = start;
        this.end = end;
        this.color = color;
    }

    public void spanField(double a, double b, double c) {
        if (a == 0) {
            // Horizontal Line
            double y = c / b;
            start = new Translation2d(0, y);
            end = new Translation2d(FieldConstants.fieldLength, y);
        } else if (b == 0) {
            // Vertical Line
            double x = c / a;
            start = new Translation2d(x, 0);
            end = new Translation2d(x, FieldConstants.fieldWidth);
        } else {
            double xstartX = 0;
            double xstartY = c / b;
            double xendX = FieldConstants.fieldLength;
            double xendY = (c - a * xendX) / b;

            start = new Translation2d(xstartX, xstartY);
            end = new Translation2d(xendX, xendY);
        }
    }



    @Override
    public void draw() {
        Translation2d[] res = new Translation2d[2];
        res[0] = start;
        res[1] = end;

        Logger.recordOutput(this.key, res);
    }

    @Override
    public ObjectNode layout(ObjectMapper om) {
        ObjectNode source = om.createObjectNode();
        source.put("type", "trajectory");
        source.put("logKey", "NT:/AdvantageKit/RealOutputs/" + this.key);
        source.put("logType", "Translation2d[]");
        source.put("visible", true);
        ObjectNode options = om.createObjectNode();
        options.put("color", color);
        options.put("size", "normal");
        source.set("options", options);
        return source;
    }
}
