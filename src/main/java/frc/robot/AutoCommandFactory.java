package frc.robot;

import static edu.wpi.first.units.Units.Seconds;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

    private Command stopRobot() {
        return swerve.runOnce(swerve::setMotorsZero).alongWith(Commands.waitTime(Seconds.of(0.01)));
    }

    public AutoRoutine resnick() {

        AutoRoutine routine = autoFactory.newRoutine("Resnick");

        Trigger haveCoral = routine.observe(coral.intakedCoralRight).or(() -> Robot.isSimulation());
        // Trigger atScoring =
        // new Trigger(() -> elevator.getHeight().isNear(Constants.Elevator.P4, Inches.of(1)));

        AutoTrajectory startToScore1 = routine.trajectory("Resnick", 0);
        AutoTrajectory Score1ToFeeder = routine.trajectory("Resnick", 1);
        AutoTrajectory FeederToScore2 = routine.trajectory("Resnick", 2);
        AutoTrajectory score2ToFeeder = routine.trajectory("Resnick", 3);
        AutoTrajectory feederToScore3 = routine.trajectory("Resnick", 4);

        routine.active()
            .onTrue(Commands.sequence(startToScore1.resetOdometry(), startToScore1.cmd()));
        startToScore1.active().onTrue(elevator.p0());
        startToScore1.done()
            .onTrue(Commands.sequence(stopRobot(), elevator.p4(),
                coral.runScoringMotor(0.5).withTimeout(1.0), elevator.p0(),
                Score1ToFeeder.cmd().asProxy()));
        Score1ToFeeder.active().onTrue(elevator.home());
        Score1ToFeeder.done().and(haveCoral)
            .onTrue(Commands.sequence(stopRobot(), FeederToScore2.cmd().asProxy()));
        FeederToScore2.active().onTrue(elevator.p0());
        FeederToScore2.done()
            .onTrue(Commands.sequence(stopRobot(), elevator.p4(),
                coral.runScoringMotor(0.5).withTimeout(1.0), elevator.p0(),
                score2ToFeeder.cmd().asProxy()));
        score2ToFeeder.active().onTrue(elevator.home());
        score2ToFeeder.done().and(haveCoral)
            .onTrue(Commands.sequence(stopRobot(), feederToScore3.cmd().asProxy()));
        feederToScore3.active().onTrue(elevator.p0());
        feederToScore3.done().onTrue(Commands.sequence(stopRobot(), elevator.p4(),
            coral.runScoringMotor(0.5).withTimeout(1.0), elevator.p0()));

        return routine;
    }
}
