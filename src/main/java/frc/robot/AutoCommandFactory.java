package frc.robot;

import java.util.Optional;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.AllianceFlipUtil;
import frc.lib.util.Container;
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
    Container<Boolean> intakingAlgae;

    private SendableChooser<ScoringLocation.Height> algaeHeight =
        new SendableChooser<ScoringLocation.Height>();

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
        CoralScoring coral, ElevatorAlgae algae, LEDs leds, Container<Boolean> intakingAlgae) {
        this.autoFactory = autoFactory;
        this.swerve = swerve;
        this.elevator = elevator;
        this.coral = coral;
        this.algae = algae;
        this.leds = leds;
        this.intakingAlgae = intakingAlgae;

        algaeHeight.setDefaultOption("High", ScoringLocation.Height.KP2);
        algaeHeight.addOption("Low", ScoringLocation.Height.KP0);
        SmartDashboard.putData(Constants.DashboardValues.algaeHeight, algaeHeight);
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

    private static final Pose2d middleStart =
        new Pose2d(7.578684329986572, 3.9847824573516846, Rotation2d.k180deg);

    /**
     * Left L4 Auto
     *
     * @return Auto Routine
     */
    public AutoRoutine l4left() {
        AutoRoutine routine = autoFactory.newRoutine("Test");
        routine.active()
            .onTrue(Commands.sequence(
                CommandFactory.autoScore(swerve, elevator, coral, algae, () -> CoralLocation.I,
                    () -> Height.KP4, () -> Optional.empty(), intakingAlgae, (x) -> {
                    }),
                CommandFactory.leftFeeder(swerve, elevator, coral),
                coral.runCoralIntake().until(coral.coralAtIntake),
                CommandFactory.autoScore(swerve, elevator, coral, algae, () -> CoralLocation.K,
                    () -> Height.KP4, () -> Optional.empty(), intakingAlgae, (x) -> {
                    }),
                CommandFactory.leftFeeder(swerve, elevator, coral),
                coral.runCoralIntake().until(coral.coralAtIntake),
                CommandFactory.autoScore(swerve, elevator, coral, algae, () -> CoralLocation.J,
                    () -> Height.KP4, () -> Optional.empty(), intakingAlgae, (x) -> {
                    }),
                CommandFactory.leftFeeder(swerve, elevator, coral),
                coral.runCoralIntake().until(coral.coralAtIntake),
                CommandFactory.autoScore(swerve, elevator, coral, algae, () -> CoralLocation.L,
                    () -> Height.KP4, () -> Optional.empty(), intakingAlgae, (x) -> {
                    }),
                CommandFactory.leftFeeder(swerve, elevator, coral),
                coral.runCoralIntake().until(coral.coralAtIntake), swerve.stop())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        return routine;
    }

    /**
     * Right L4 Auto
     *
     * @return Auto Routine
     */
    public AutoRoutine l4right() {
        AutoRoutine routine = autoFactory.newRoutine("Test2");
        routine.active()
            .onTrue(Commands.sequence(
                CommandFactory.autoScore(swerve, elevator, coral, algae, () -> CoralLocation.E,
                    () -> Height.KP4, () -> Optional.empty(), intakingAlgae, (x) -> {
                    }),
                CommandFactory.rightFeeder(swerve, elevator, coral),
                coral.runCoralIntake().until(coral.coralAtIntake),
                CommandFactory.autoScore(swerve, elevator, coral, algae, () -> CoralLocation.C,
                    () -> Height.KP4, () -> Optional.empty(), intakingAlgae, (x) -> {
                    }),
                CommandFactory.rightFeeder(swerve, elevator, coral),
                coral.runCoralIntake().until(coral.coralAtIntake),
                CommandFactory.autoScore(swerve, elevator, coral, algae, () -> CoralLocation.F,
                    () -> Height.KP4, () -> Optional.empty(), intakingAlgae, (x) -> {
                    }),
                CommandFactory.rightFeeder(swerve, elevator, coral),
                coral.runCoralIntake().until(coral.coralAtIntake),
                CommandFactory.autoScore(swerve, elevator, coral, algae, () -> CoralLocation.D,
                    () -> Height.KP4, () -> Optional.empty(), intakingAlgae, (x) -> {
                    }),
                CommandFactory.rightFeeder(swerve, elevator, coral),
                coral.runCoralIntake().until(coral.coralAtIntake), swerve.stop())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        return routine;
    }

    /** Barge auto */
    public AutoRoutine barge() {

        AutoRoutine routine = autoFactory.newRoutine("Barge");

        routine.active().onTrue(Commands.sequence(swerve.runOnce(() -> {
            swerve.resetOdometry(AllianceFlipUtil.apply(middleStart));
        }), CommandFactory.autoScore(swerve, elevator, coral, algae, () -> CoralLocation.H,
            () -> algaeHeight.getSelected(), () -> Optional.empty(), intakingAlgae, (x) -> {
            }), CommandFactory.autoScore(swerve, elevator, coral, algae, () -> CoralLocation.H,
                () -> Height.KP4, () -> Optional.empty(), intakingAlgae, (x) -> {
                }),
            CommandFactory.barge(swerve, elevator),
            elevator.moveTo(() -> ScoringLocation.Height.KP5.height),
            algae.runAlgaeMotor(Constants.Algae.VOLTAGE).withDeadline(Commands.waitSeconds(.25)),
            CommandFactory.ensureHome(elevator)
                .andThen(CommandFactory.autoScore(swerve, elevator, coral, algae,
                    () -> CoralLocation.J, () -> algaeHeight.getSelected(), () -> Optional.empty(),
                    intakingAlgae, (x) -> {
                    })),
            CommandFactory.ensureHome(elevator).andThen(CommandFactory.barge(swerve, elevator)),
            elevator.moveTo(() -> ScoringLocation.Height.KP5.height),
            algae.runAlgaeMotor(Constants.Algae.VOLTAGE).withDeadline(Commands.waitSeconds(.5)),
            CommandFactory.ensureHome(elevator), swerve.stop()));
        return routine;
    }

}
