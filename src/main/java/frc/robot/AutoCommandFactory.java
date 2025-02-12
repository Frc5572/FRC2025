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
}
