package frc.robot.commands;

import java.util.function.Supplier;
import choreo.auto.AutoRoutine;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.AllianceFlipUtil;
import frc.lib.util.Tuples.Tuple2;
import frc.robot.subsystems.swerve.Swerve;

/**
 * Move to Pose2d
 */
public class MoveToPoseWithLocalTag extends Command {

    private EventLoop eventLoop = CommandScheduler.getInstance().getDefaultButtonLoop();
    private AutoRoutine autoRoutine;
    private Swerve swerve;
    private Supplier<Tuple2<Pose2d, Integer>> pose2dAndTagSupplier;
    private Pose2d pose2d;
    private boolean flipForRed;
    private double tol;
    private double rotTol;
    /** If this trajectory us currently running */
    private boolean isActive = false;
    /** If the trajectory ran to completion */
    private boolean isCompleted = false;

    /**
     * Move to a specified Pose2d command
     *
     * @param swerve Swerve Subsystem
     * @param pose2dAndTagSupplier Pose2d and tag id Supplier
     * @param flipForRed Whether to flip the pose2d for red alliance
     * @param tol Translational Tolerance
     * @param rotTol Rotational Tolerance
     */
    public MoveToPoseWithLocalTag(Swerve swerve,
        Supplier<Tuple2<Pose2d, Integer>> pose2dAndTagSupplier, boolean flipForRed, double tol,
        double rotTol) {
        this.swerve = swerve;
        this.pose2dAndTagSupplier = pose2dAndTagSupplier;
        this.flipForRed = flipForRed;
        this.tol = tol;
        this.rotTol = rotTol;
        addRequirements(swerve);
    }

    /**
     * Move to a specified Pose2d command
     *
     * @param swerve Swerve Subsystem
     * @param pose2dAndTagSupplier Pose2d and tag id Supplier
     * @param flipForRed Whether to flip the pose2d for red alliance
     * @param tol Translational Tolerance
     * @param rotTol Rotational Tolerance
     * @param autoRoutine Choreo AutoRoutine to integrate command
     */
    public MoveToPoseWithLocalTag(Swerve swerve,
        Supplier<Tuple2<Pose2d, Integer>> pose2dAndTagSupplier, boolean flipForRed, double tol,
        double rotTol, AutoRoutine autoRoutine) {
        this(swerve, pose2dAndTagSupplier, flipForRed, tol, rotTol);
        this.autoRoutine = autoRoutine;
        this.eventLoop = autoRoutine.loop();
    }

    /**
     * Returns a trigger that is true while the trajectory is scheduled.
     *
     * @return A trigger that is true while the trajectory is scheduled.
     */
    public Trigger active() {
        if (autoRoutine != null) {
            return new Trigger(eventLoop,
                () -> this.isActive && autoRoutine.active().getAsBoolean());
        }
        return new Trigger(eventLoop, () -> this.isActive);
    }

    public Trigger done() {
        return new Trigger(eventLoop, () -> isCompleted);
    }

    @Override
    public void initialize() {
        isActive = true;
        isCompleted = false;
        var x = pose2dAndTagSupplier.get();
        pose2d = x._0();
        if (flipForRed) {
            pose2d = AllianceFlipUtil.apply(pose2d);
            swerve.state.setLocalTarget(AllianceFlipUtil.applyAprilTag(x._1()));
        } else {
            swerve.state.setLocalTarget(x._1());
        }
    }

    @Override
    public void execute() {
        swerve.moveToPose(pose2d);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setMotorsZero();
        swerve.state.setLocalTarget(0);
        isActive = false;
        isCompleted = !interrupted;
    }

    @Override
    public boolean isFinished() {
        Pose2d poseError = Pose2d.kZero.plus(pose2d.minus(swerve.getPose()));
        final var eTranslate = poseError.getTranslation();
        final var eRotate = poseError.getRotation();
        return Math.abs(eTranslate.getX()) < tol && Math.abs(eTranslate.getY()) < tol
            && Math.abs(eRotate.getDegrees()) < rotTol;
    }


}
