package frc.robot;

import static edu.wpi.first.units.Units.Seconds;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.coral.CoralScoring;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.swerve.Swerve;

public class AutoCommandFactory {

    AutoFactory autoFactory;
    Swerve swerve;
    Elevator elevator;
    CoralScoring coral;

    public AutoCommandFactory(AutoFactory autoFactory, Swerve swerve, Elevator elevator,
        CoralScoring coral) {
        this.autoFactory = autoFactory;
        this.swerve = swerve;
        this.elevator = elevator;
        this.coral = coral;
    }

    public AutoRoutine resnick() {
        AutoRoutine routine = autoFactory.newRoutine("Resnick");

        Trigger haveCoral = routine.observe(coral.intakedCoralRight);

        AutoTrajectory startToScore1 = routine.trajectory("Resnick", 0);
        AutoTrajectory Score1ToFeeder = routine.trajectory("Resnick", 1);
        AutoTrajectory FeederToScore2 = routine.trajectory("Resnick", 2);

        routine.active()
            .onTrue(Commands.sequence(startToScore1.resetOdometry(), startToScore1.cmd()));
        startToScore1.active().onTrue(elevator.p0());
        startToScore1.done()
            .onTrue(Commands.sequence(
                swerve.runOnce(swerve::setMotorsZero)
                    .alongWith(Commands.waitTime(Seconds.of(0.01))),
                elevator.p4(), coral.runScoringMotor(0.5), elevator.p0(),
                new ProxyCommand(Score1ToFeeder.cmd())));
        Score1ToFeeder.active().onTrue(elevator.home());
        Score1ToFeeder.done().onTrue(Commands.sequence(
            swerve.runOnce(swerve::setMotorsZero).alongWith(Commands.waitTime(Seconds.of(0.01))),
            new ProxyCommand(FeederToScore2.cmd())));
        FeederToScore2.done().onTrue(
            swerve.runOnce(swerve::setMotorsZero).alongWith(Commands.waitTime(Seconds.of(0.01))));

        return routine;
    }

    public AutoRoutine watson() {
        AutoRoutine routine = autoFactory.newRoutine("Watson");

        Trigger haveCoral = routine.observe(coral.intakedCoralRight);

        AutoTrajectory startToScore1 = routine.trajectory("Watson1", 0);
        AutoTrajectory Score1ToFeeder = routine.trajectory("Watson2", 0);
        AutoTrajectory FeederToScore2 = routine.trajectory("Watson3", 0);
        AutoTrajectory Score2ToFeeder = routine.trajectory("Watson4", 0);

        routine.active()
            .onTrue(Commands.sequence(startToScore1.resetOdometry(), startToScore1.cmd()));
        startToScore1.active().onTrue(elevator.p0());
        startToScore1.done().onTrue(Commands.sequence(
            swerve.moveToPose(() -> Score1ToFeeder.getInitialPose().get(), false, 0.1, 1)
                .withTimeout(1),
            swerve.runOnce(swerve::setMotorsZero).alongWith(Commands.waitTime(Seconds.of(0.01))),
            elevator.p4(), coral.runScoringMotor(0.5), elevator.p0(),
            new ProxyCommand(Score1ToFeeder.cmd())));
        Score1ToFeeder.active().onTrue(Commands.waitSeconds(0.5).andThen(elevator.home()));
        Score1ToFeeder.done().onTrue(Commands.sequence(
            swerve.moveToPose(() -> FeederToScore2.getInitialPose().get(), false, 0.1, 1)
                .withTimeout(1),
            swerve.runOnce(swerve::setMotorsZero).alongWith(Commands.waitTime(Seconds.of(0.01))),
            Commands.waitSeconds(2), new ProxyCommand(FeederToScore2.cmd())));
        FeederToScore2.active().onTrue(Commands.waitSeconds(0.5).andThen(elevator.p0()));
        FeederToScore2.done().onTrue(Commands.sequence(
            swerve.moveToPose(() -> Score2ToFeeder.getInitialPose().get(), false, 0.1, 1)
                .withTimeout(1),
            swerve.runOnce(swerve::setMotorsZero).alongWith(Commands.waitTime(Seconds.of(0.01))),
            elevator.p4(), coral.runScoringMotor(0.5), elevator.p0(),
            new ProxyCommand(Score2ToFeeder.cmd())));
        Score2ToFeeder.active().onTrue(Commands.waitSeconds(0.5).andThen(elevator.home()));
        Score2ToFeeder.done().onTrue(Commands.sequence(
            swerve.moveToPose(() -> FeederToScore2.getInitialPose().get(), false, 0.1, 1)
                .withTimeout(1),
            swerve.runOnce(swerve::setMotorsZero).alongWith(Commands.waitTime(Seconds.of(0.01))),
            Commands.waitSeconds(2), new ProxyCommand(FeederToScore2.cmd())));

        return routine;
    }

    public AutoRoutine test() {
        AutoRoutine routine = autoFactory.newRoutine("test");

        // Trigger haveCoral = routine.observe(coral.intakedCoralRight);

        AutoTrajectory part1 = routine.trajectory("Test", 0);
        AutoTrajectory part2 = routine.trajectory("Test", 1);
        AutoTrajectory part3 = routine.trajectory("Test", 2);
        AutoTrajectory part4 = routine.trajectory("Test", 3);

        routine.active().onTrue(Commands.sequence(part1.resetOdometry(), part1.cmd()));
        part1.done().onTrue(part2.cmd());
        part2.done().onTrue(Commands.sequence(swerve.runOnce(swerve::setMotorsZero),
            Commands.waitSeconds(3), part3.cmd()));
        // part3.done().onTrue(part4.cmd());


        return routine;
    }

    public AutoRoutine middleBot() {
        AutoRoutine routine = autoFactory.newRoutine("middleBot");

        // Trigger haveCoral = routine.observe(coral.intakedCoralRight);

        AutoTrajectory part1 = routine.trajectory("middleBot", 0);
        AutoTrajectory part2 = routine.trajectory("middleBot", 1);
        AutoTrajectory part3 = routine.trajectory("middleBot", 2);
        AutoTrajectory part4 = routine.trajectory("middleBot", 3);
        AutoTrajectory part5 = routine.trajectory("middleBot", 4);
        AutoTrajectory part6 = routine.trajectory("middleBot", 5);
        AutoTrajectory part7 = routine.trajectory("middleBot", 6);
        AutoTrajectory part8 = routine.trajectory("middleBot", 7);
        AutoTrajectory part9 = routine.trajectory("middleBot", 8);
        AutoTrajectory part10 = routine.trajectory("middleBot", 9);
        AutoTrajectory part11 = routine.trajectory("middleBot", 10);
        AutoTrajectory part12 = routine.trajectory("middleBot", 11);
        AutoTrajectory part13 = routine.trajectory("middleBot", 12);
        AutoTrajectory part14 = routine.trajectory("middleBot", 13);
        // AutoTrajectory part15 = routine.trajectory("middleBot", 14);
        // AutoTrajectory part16 = routine.trajectory("middleBot", 15);


        routine.active().onTrue(Commands.sequence(part1.resetOdometry(), part1.cmd()));
        part1.done().onTrue(Commands.sequence(swerve.runOnce(swerve::setMotorsZero),
            Commands.waitSeconds(0.5), elevator.p0(), Commands.waitSeconds(0.2), part2.cmd()));
        part1.active().onTrue(elevator.p4());
        // part2.done().onTrue(Commands.sequence(swerve.runOnce(swerve::setMotorsZero),
        // Commands.waitSeconds(3), part3.cmd()));
        part2.done().onTrue(part3.cmd());
        // part2.active().onTrue(elevator.p0());
        part3.done().onTrue(part4.cmd());
        part4.done().onTrue(part5.cmd());
        part5.done().onTrue(part6.cmd());
        part6.done().onTrue(part7.cmd());
        part7.done().onTrue(part8.cmd());
        part8.done().onTrue(part9.cmd());
        part9.done().onTrue(part10.cmd());
        part10.done().onTrue(part11.cmd());
        part11.done().onTrue(part12.cmd());
        part12.done().onTrue(part13.cmd());
        part13.done().onTrue(part14.cmd());
        part14.done().onTrue(Commands.sequence(swerve.runOnce(swerve::setMotorsZero)));
        // part15.done().onTrue(Commands.sequence(swerve.runOnce(swerve::setMotorsZero)));
        // part2.done().onTrue(Commands.sequence(swerve.runOnce(swerve::setMotorsZero),
        // Commands.waitSeconds(3), part3.cmd()));



        return routine;
    }
}
