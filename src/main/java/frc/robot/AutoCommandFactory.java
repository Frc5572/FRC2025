package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.commands.MoveToPose;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.coral.CoralScoring;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.swerve.Swerve;

/**
 * Command Factory for Autos
 */
public class AutoCommandFactory {

    AutoFactory autoFactory;
    Swerve swerve;
    Elevator elevator;
    CoralScoring coral;
    LEDs leds;

    /**
     * Command Factory for Autos
     *
     * @param autoFactory Choreo Auto Factory
     * @param swerve Swerve Subsystem
     * @param elevator Elevator Subsystem
     * @param coral Coral Subsystem
     * @param leds LED Subsystem
     */
    public AutoCommandFactory(AutoFactory autoFactory, Swerve swerve, Elevator elevator,
        CoralScoring coral, LEDs leds) {
        this.autoFactory = autoFactory;
        this.swerve = swerve;
        this.elevator = elevator;
        this.coral = coral;
        this.leds = leds;
    }

    /**
     * Example Auto Routine
     *
     * @return Auto Routine
     */
    public AutoRoutine example() {
        AutoRoutine routine = autoFactory.newRoutine("Example");
        MoveToPose testMTP =
            new MoveToPose(swerve,
                () -> new Pose2d(FieldConstants.Reef.center.getX(),
                    FieldConstants.Barge.middleCage.getY(), new Rotation2d()),
                true, .25, 10, routine);

        routine.active()
            .onTrue(Commands.sequence(
                Commands.runOnce(() -> swerve
                    .resetOdometry(new Pose2d(FieldConstants.Barge.middleCage, new Rotation2d()))),
                leds.blinkLEDs(Color.kGreen, 5), testMTP));
        testMTP.done().onTrue(leds.blinkLEDs(Color.kPurple, 5));
        return routine;
    }

    public AutoRoutine jacetest() {
        AutoRoutine routine = autoFactory.newRoutine("jacetest");

        AutoTrajectory start = routine.trajectory("jaceTest1", 0);
        AutoTrajectory goToReef = routine.trajectory("jaceTest2", 0);
        // AutoTrajectory lowerElevator1 = routine.trajectory("jaceTest3", 0);
        AutoTrajectory feed1 = routine.trajectory("jaceTest4", 0);
        AutoTrajectory goToReef2 = routine.trajectory("jaceTest5", 0);
        // AutoTrajectory lowerElevator2 = routine.trajectory("jaceTest6", 0);
        AutoTrajectory feed2 = routine.trajectory("jaceTest7", 0);
        AutoTrajectory goToReef3 = routine.trajectory("jaceTest8", 0);
        // AutoTrajectory lowerElevator3 = routine.trajectory("jaceTest9", 0);
        AutoTrajectory feed4 = routine.trajectory("jaceTest9", 0);
        AutoTrajectory goToReef4 = routine.trajectory("jaceTest10", 0);

        routine.active().onTrue(Commands.sequence(start.resetOdometry(), start.cmd()));
        start.active().onChange(elevator.home());
        start.done()
            .onTrue(Commands.sequence(
                Commands.runOnce(() -> swerve.moveToPose(goToReef.getInitialPose().get())),
                swerve.runOnce(swerve::setMotorsZero), elevator.p4(),
                new ProxyCommand(goToReef.cmd())));
        goToReef.active().onChange(elevator.p4());
        goToReef.done()
            .onTrue(Commands.sequence(
                Commands.runOnce(() -> swerve.moveToPose(feed1.getInitialPose().get())),
                swerve.runOnce(swerve::setMotorsZero), elevator.p4(), coral.runCoralOuttake(),
                new ProxyCommand(feed1.cmd())));
        feed1.active().onTrue(elevator.p4());
        feed1.done()
            .onTrue(Commands.sequence(
                Commands.runOnce(() -> swerve.moveToPose(feed1.getInitialPose().get())),
                swerve.runOnce(swerve::setMotorsZero), elevator.home(),
                new ProxyCommand(feed1.cmd())));;
        feed1.active().onTrue(elevator.home());
        feed1.done()
            .onTrue(Commands.sequence(
                Commands.runOnce(() -> swerve.moveToPose(goToReef2.getInitialPose().get())),
                swerve.runOnce(swerve::setMotorsZero), elevator.p4(),
                new ProxyCommand(goToReef2.cmd())));;
        goToReef2.active().onTrue(elevator.p4());
        goToReef2.done()
            .onTrue(Commands.sequence(
                Commands.runOnce(() -> swerve.moveToPose(feed2.getInitialPose().get())),
                swerve.runOnce(swerve::setMotorsZero), elevator.p4(), coral.runCoralOuttake(),
                new ProxyCommand(feed2.cmd())));;
        feed2.active().onTrue(elevator.p4());
        feed2.done()
            .onTrue(Commands.sequence(
                Commands.runOnce(() -> swerve.moveToPose(feed2.getInitialPose().get())),
                swerve.runOnce(swerve::setMotorsZero), elevator.home(),
                new ProxyCommand(feed2.cmd())));;
        feed2.active().onTrue(elevator.home());
        feed2.done()
            .onTrue(Commands.sequence(
                Commands.runOnce(() -> swerve.moveToPose(goToReef3.getInitialPose().get())),
                swerve.runOnce(swerve::setMotorsZero), elevator.p4(), coral.runCoralOuttake(),
                new ProxyCommand(goToReef3.cmd())));
        goToReef3.active().onTrue(elevator.p4());
        goToReef3.done().onTrue(Commands.sequence(
            Commands.runOnce(() -> swerve.moveToPose(feed4.getInitialPose().get())),
            swerve.runOnce(swerve::setMotorsZero), elevator.p4(), new ProxyCommand(feed4.cmd())));
        feed4.active().onTrue(elevator.home());
        feed4.done()
            .onTrue(Commands.sequence(
                Commands.runOnce(() -> swerve.moveToPose(goToReef4.getInitialPose().get())),
                swerve.runOnce(swerve::setMotorsZero), elevator.p4(),
                new ProxyCommand(goToReef4.cmd())));;;
        goToReef4.active().onTrue(elevator.p4());
        goToReef4.done();

        return routine;
    }
}
