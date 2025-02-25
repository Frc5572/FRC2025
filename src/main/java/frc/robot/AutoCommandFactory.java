package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Commands;
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

    public AutoRoutine HIJK() {
        AutoRoutine routine = autoFactory.newRoutine("HIJK");

        AutoTrajectory StartToH = routine.trajectory("Reed", 0);
        AutoTrajectory HToFeeder = routine.trajectory("Reed", 1);
        AutoTrajectory feederToIPrep = routine.trajectory("Reed", 2);
        AutoTrajectory IprepToI = routine.trajectory("Reed", 3);
        AutoTrajectory IToFeeder = routine.trajectory("Reed", 4);
        AutoTrajectory FeederToJPrep = routine.trajectory("Reed", 5);

        return routine;
    }

    public AutoRoutine CoralAndBarge() {
        AutoRoutine routine = autoFactory.newRoutine("CoralAndBarge");
        return routine;
    }
}
