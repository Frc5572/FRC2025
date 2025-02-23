package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import choreo.auto.AutoRoutine;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.AllianceFlipUtil;
import frc.lib.util.Tuples.Tuple2;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.Swerve;

/**
 * Move to Pose2d
 */
public class MoveToPoseWithLocalTag extends Command {

    private EventLoop eventLoop = CommandScheduler.getInstance().getDefaultButtonLoop();
    private AutoRoutine autoRoutine;
    private final Swerve swerve;
    private final Supplier<Tuple2<Pose2d, Integer>> pose2dAndTagSupplier;
    private final DoubleSupplier maxSpeedSupplier;
    private Pose2d pose2d;
    private final boolean flipForRed;
    private final double tol;
    private final double rotTol;
    /** If this trajectory us currently running */
    private boolean isActive = false;
    /** If the trajectory ran to completion */
    private boolean isCompleted = false;

    /**
     * Move to a specified Pose2d command
     *
     * @param swerve Swerve Subsystem
     * @param pose2dAndTagSupplier Pose2d and tag id Supplier
     * @param maxSpeedSupplier maximum speed to move at
     * @param flipForRed Whether to flip the pose2d for red alliance
     * @param tol Translational Tolerance
     * @param rotTol Rotational Tolerance
     */
    public MoveToPoseWithLocalTag(Swerve swerve,
        Supplier<Tuple2<Pose2d, Integer>> pose2dAndTagSupplier, DoubleSupplier maxSpeedSupplier,
        boolean flipForRed, double tol, double rotTol) {
        this.swerve = swerve;
        this.pose2dAndTagSupplier = pose2dAndTagSupplier;
        this.maxSpeedSupplier = maxSpeedSupplier;
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
     * @param maxSpeedSupplier maximum speed to move at
     * @param flipForRed Whether to flip the pose2d for red alliance
     * @param tol Translational Tolerance
     * @param rotTol Rotational Tolerance
     * @param autoRoutine Choreo AutoRoutine to integrate command
     */
    public MoveToPoseWithLocalTag(Swerve swerve,
        Supplier<Tuple2<Pose2d, Integer>> pose2dAndTagSupplier, DoubleSupplier maxSpeedSupplier,
        boolean flipForRed, double tol, double rotTol, AutoRoutine autoRoutine) {
        this(swerve, pose2dAndTagSupplier, maxSpeedSupplier, flipForRed, tol, rotTol);
        this.autoRoutine = autoRoutine;
        this.eventLoop = autoRoutine.loop();
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
        this(swerve, pose2dAndTagSupplier, () -> Constants.Swerve.maxSpeed, flipForRed, tol,
            rotTol);
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
        finishCycleCount = 0;
    }

    @Override
    public void execute() {
        swerve.moveToPose(pose2d, maxSpeedSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setMotorsZero();
        swerve.state.setLocalTarget(0);
        isActive = false;
        isCompleted = !interrupted;
    }

    private int finishCycleCount = 0;

    @Override
    public boolean isFinished() {
        Pose2d poseError = Pose2d.kZero.plus(pose2d.minus(swerve.getPose()));
        final var eTranslate = poseError.getTranslation();
        final var eRotate = poseError.getRotation();
        Logger.recordOutput("moveToPose/eTranslate", eTranslate);
        Logger.recordOutput("moveToPose/eTranslateNorm", eTranslate.getNorm());
        Logger.recordOutput("moveToPose/eRotate", eRotate);
        if (eTranslate.getNorm() < tol && Math.abs(eRotate.getDegrees()) < rotTol) {
            finishCycleCount += 1;
        } else {
            finishCycleCount = 0;
        }
        return finishCycleCount > 5;
    }


}
