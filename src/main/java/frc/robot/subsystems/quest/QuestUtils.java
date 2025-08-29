package frc.robot.subsystems.quest;

import org.ejml.simple.SimpleMatrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class QuestUtils {
    private SimpleMatrix x; // state [x, y, theta]
    private SimpleMatrix P; // covariance
    private SimpleMatrix Q; // process noise

    public QuestUtils() {
        x = new SimpleMatrix(3, 1); // initialize at origin
        P = SimpleMatrix.identity(3).scale(0.05); // initial covariance
        Q = SimpleMatrix.identity(3).scale(0.01); // small process noise
    }

    /** Prediction step using QuestNav */
    public void predict(Pose2d questPose) {
        x.set(0, questPose.getX());
        x.set(1, questPose.getY());
        x.set(2, questPose.getRotation().getRadians());

        // Covariance prediction
        P = P.plus(Q);
    }

    /** Adaptive R based on distance to tag, returns a diagonal SimpleMatrix */
    public SimpleMatrix getAdaptiveR(Pose2d tagPose) {
        double dx = tagPose.getX() - x.get(0);
        double dy = tagPose.getY() - x.get(1);
        double distance = Math.hypot(dx, dy);

        double minXY = 0.02;
        double maxXY = 0.2;
        double minTheta = 0.05;

        double r_xy = Math.min(minXY + 0.03 * distance, maxXY);
        double r_theta = minTheta + 0.01 * distance;

        return SimpleMatrix.diag(r_xy * r_xy, r_xy * r_xy, r_theta * r_theta);
    }

    /** Update step using AprilTag measurement */
    public void update(Pose2d tagPose) {
        SimpleMatrix z = new SimpleMatrix(3, 1);
        z.set(0, tagPose.getX());
        z.set(1, tagPose.getY());
        z.set(2, tagPose.getRotation().getRadians());

        SimpleMatrix H = SimpleMatrix.identity(3);
        SimpleMatrix R = getAdaptiveR(tagPose);

        SimpleMatrix y = z.minus(H.mult(x));
        SimpleMatrix S = H.mult(P).mult(H.transpose()).plus(R);
        SimpleMatrix K = P.mult(H.transpose()).mult(S.invert());

        x = x.plus(K.mult(y));
        P = (SimpleMatrix.identity(3).minus(K.mult(H))).mult(P);
    }

    /** Get current fused pose */
    public Pose2d getPose() {
        return new Pose2d(x.get(0), x.get(1), new Rotation2d(x.get(2)));
    }
}
