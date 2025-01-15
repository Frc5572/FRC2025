
package frc.lib.viz;

import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.sim.SimulatedRobot;

/**
 * Robot Viz
 */
public class RobotViz {

    private final String prefix;
    private final SimulatedRobot sim;

    /**
     * Pumba Viz
     *
     * @param prefix Prefix
     * @param sim Simulation
     */
    public RobotViz(String prefix, SimulatedRobot sim) {
        this.prefix = prefix;
        this.sim = sim;

    }


    /**
     * Set Robot Pose
     *
     * @param pose Position of robot
     */
    public void setPose(Pose2d pose) {}

    /**
     * Note Location
     */
    public static enum NoteLocation {
        Intake, Shooter, None
    }



    private static final double SHOOT_SPEED = 33.0;

    void recalculateTrajectory() {

    }

}
