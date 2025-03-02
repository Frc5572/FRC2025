package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import choreo.auto.AutoRoutine;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.AllianceFlipUtil;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.swerve.Swerve;

/**
 * Move to Pose2d
 */
public class MoveToPose extends Command {

    private EventLoop eventLoop = CommandScheduler.getInstance().getDefaultButtonLoop();
    private AutoRoutine autoRoutine;
    private final Swerve swerve;
    private final Elevator elevator;
    private final Supplier<Pose2d> pose2dSupplier;
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
     * @param pose2dSupplier Pose2d Supplier
     * @param maxSpeedSupplier maximum speed to move at
     * @param flipForRed Whether to flip the pose2d for red alliance
     * @param tol Translational Tolerance
     * @param rotTol Rotational Tolerance
     */
    public MoveToPose(Swerve swerve, Elevator elevator, Supplier<Pose2d> pose2dSupplier,
        DoubleSupplier maxSpeedSupplier, boolean flipForRed, double tol, double rotTol) {
        this.swerve = swerve;
        this.elevator = elevator;
        this.pose2dSupplier = pose2dSupplier;
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
     * @param pose2dSupplier Pose2d Supplier
     * @param maxSpeedSupplier maximum speed to move at
     * @param flipForRed Whether to flip the pose2d for red alliance
     * @param tol Translational Tolerance
     * @param rotTol Rotational Tolerance
     * @param autoRoutine Choreo AutoRoutine to integrate command
     */
    public MoveToPose(Swerve swerve, Elevator elevator, Supplier<Pose2d> pose2dSupplier,
        DoubleSupplier maxSpeedSupplier, boolean flipForRed, double tol, double rotTol,
        AutoRoutine autoRoutine) {
        this(swerve, elevator, pose2dSupplier, maxSpeedSupplier, flipForRed, tol, rotTol);
        this.autoRoutine = autoRoutine;
        this.eventLoop = autoRoutine.loop();
    }

    /**
     * Move to a specified Pose2d command
     *
     * @param swerve Swerve Subsystem
     * @param pose2dSupplier Pose2d Supplier
     * @param flipForRed Whether to flip the pose2d for red alliance
     * @param tol Translational Tolerance
     * @param rotTol Rotational Tolerance
     * @param autoRoutine Choreo AutoRoutine to integrate command
     */
    public MoveToPose(Swerve swerve, Elevator elevator, Supplier<Pose2d> pose2dSupplier,
        boolean flipForRed, double tol, double rotTol, AutoRoutine autoRoutine) {
        this(swerve, elevator, pose2dSupplier, () -> Constants.Swerve.maxSpeed, flipForRed, tol,
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
        pose2d = pose2dSupplier.get();
        if (flipForRed) {
            pose2d = AllianceFlipUtil.apply(pose2d);
        }
    }

    @Override
    public void execute() {
        swerve.moveToPose(pose2d, elevator.hightAboveP0(), maxSpeedSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setMotorsZero();
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
