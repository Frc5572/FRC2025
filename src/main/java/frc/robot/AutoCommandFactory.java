package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.AllianceFlipUtil;
import frc.lib.util.ScoringLocation;
import frc.lib.util.ScoringLocation.CoralLocation;
import frc.lib.util.ScoringLocation.Height;
import frc.robot.commands.MoveToPose;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.coral.CoralScoring;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator_algae.ElevatorAlgae;
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
    ElevatorAlgae algae;

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
        CoralScoring coral, ElevatorAlgae algae, LEDs leds) {
        this.autoFactory = autoFactory;
        this.swerve = swerve;
        this.elevator = elevator;
        this.coral = coral;
        this.algae = algae;
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

    private static final Pose2d leftStart = new Pose2d(7.50, 6.12, Rotation2d.k180deg);
    private static final Pose2d middleStart =
        new Pose2d(7.578684329986572, 3.9847824573516846, Rotation2d.k180deg);

    /**
     * Example Auto Routine
     *
     * @return Auto Routine
     */
    public AutoRoutine test() {
        AutoRoutine routine = autoFactory.newRoutine("Test");

        routine.active().onTrue(Commands.sequence(swerve.runOnce(() -> {
            swerve.resetOdometry(AllianceFlipUtil.apply(leftStart));
        }), CommandFactory.autoScore(swerve, elevator, coral, algae, () -> CoralLocation.A, () -> {
            if (SmartDashboard.getBoolean("isHigh", false)) {
                return Height.KP2;
            } else {
                return Height.KP0;
            }
        }), CommandFactory.leftFeeder(swerve, elevator), Commands.waitSeconds(1.0),
            CommandFactory.autoScore(swerve, elevator, coral, algae, () -> CoralLocation.K,
                () -> Height.KP4),
            CommandFactory.leftFeeder(swerve, elevator), Commands.waitSeconds(1.0),
            CommandFactory.autoScore(swerve, elevator, coral, algae, () -> CoralLocation.J,
                () -> Height.KP4),
            CommandFactory.leftFeeder(swerve, elevator), Commands.waitSeconds(1.0),
            CommandFactory.autoScore(swerve, elevator, coral, algae, () -> CoralLocation.L,
                () -> Height.KP4),
            CommandFactory.leftFeeder(swerve, elevator), Commands.waitSeconds(1.0), swerve.stop()));
        return routine;
    }

    boolean dashboard = SmartDashboard.putBoolean("isHigh", true);

    public AutoRoutine barge() {
        boolean AlgaeIsHigh = SmartDashboard.getBoolean("isHigh", false);

        AutoRoutine routine = autoFactory.newRoutine("Barge");
//@formatter:off
        routine.active().onTrue(Commands.sequence(swerve.runOnce(() -> {
            swerve.resetOdometry(AllianceFlipUtil.apply(middleStart)); //start at middle
        }), CommandFactory.autoScore(swerve, elevator, coral, algae, () -> CoralLocation.H, () -> { //take off algae in back
            return (AlgaeIsHigh) ? Height.KP2 : Height.KP0;
        }), CommandFactory.autoScore(swerve, elevator, coral, algae, () -> CoralLocation.H, //score lvl 4 coral
            () -> Height.KP4), CommandFactory.barge(swerve, elevator), //travel to barge
            elevator.moveTo(() -> ScoringLocation.Height.KP5.height), //move elevator up to barge
            algae.runAlgaeMotor(Constants.Algae.VOLTAGE) //run algae motor for 1 second
                .withDeadline(Commands.waitSeconds(.25)),CommandFactory.ensureHome(elevator).andThen( //go home
            CommandFactory.autoScore(swerve, elevator, coral, algae, () -> CoralLocation.J, () -> { //go to j and grap algae
                return (AlgaeIsHigh) ? Height.KP0 : Height.KP2;
            })),CommandFactory.ensureHome(elevator).andThen(CommandFactory.barge(swerve, elevator)), // go home then travel to barge
            elevator.moveTo(() -> ScoringLocation.Height.KP5.height),// move elevator to barge
            algae.runAlgaeMotor(Constants.Algae.VOLTAGE).withDeadline(Commands.waitSeconds(.5)), CommandFactory.ensureHome(elevator), swerve.stop())); //score
        return routine;
    }
    /*
     * start center
     * go to h and grab algae
     * score lvl 4 h
     * elevator down
     * go to barge and score
     * go to j and grab algae
     * elevator down
     * go to barge and score
     */
    // @formatter:on
}
