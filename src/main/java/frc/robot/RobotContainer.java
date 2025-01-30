package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.util.viz.FieldViz;
import frc.lib.util.viz.Viz2025;
import frc.robot.Robot.RobotRunType;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorIO;
import frc.robot.subsystems.Elevator.ElevatorReal;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveIO;
import frc.robot.subsystems.swerve.SwerveReal;
import frc.robot.subsystems.swerve.SwerveSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionReal;
import frc.robot.subsystems.vision.VisionSimPhoton;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    /* Controllers */
    public final CommandXboxController driver = new CommandXboxController(Constants.driverId);

    /** Simulation */
    private SwerveDriveSimulation driveSimulation;


    /** Visualization */
    private final FieldViz fieldVis;
    private final Viz2025 vis;
    /** State */
    private final RobotState state;


    /* Subsystems */
    private final Swerve s_Swerve;
    private final Vision s_Vision;

    private Elevator elevator;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer(RobotRunType runtimeType) {
        fieldVis = new FieldViz();
        vis = new Viz2025(fieldVis, "");
        state = new RobotState(vis);
        switch (runtimeType) {
            case kReal:

                elevator = new Elevator(new ElevatorReal());

                s_Swerve = new Swerve(state, new SwerveReal());
                s_Vision = new Vision(state, VisionReal::new);
                break;
            case kSimulation:
                driveSimulation = new SwerveDriveSimulation(Constants.Swerve.getMapleConfig(),
                    new Pose2d(3, 3, Rotation2d.kZero));
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
                s_Swerve = new Swerve(state, new SwerveSim(driveSimulation));
                s_Vision = new Vision(state, VisionSimPhoton.partial(driveSimulation));
                break;
            default:

                elevator = new Elevator(new ElevatorIO() {});

                s_Swerve = new Swerve(state, new SwerveIO.Empty() {});
                s_Vision = new Vision(state, VisionIO::empty);

        }
        s_Swerve.setDefaultCommand(s_Swerve.teleOpDrive(driver, Constants.Swerve.isFieldRelative,
            Constants.Swerve.isOpenLoop));
        configureButtonBindings(runtimeType);
    }

    /**
     * Use this method to vol your button->command mappings. Buttons can be created by instantiating
     * a {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or
     * {@link XboxController}), and then passing it to a
     * {@link edu1.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings(RobotRunType runtimeType) {
        driver.y().onTrue(new InstantCommand(() -> s_Swerve.resetFieldRelativeOffset()));

        driver.povDown().onTrue(elevator.home());
        driver.povLeft().onTrue(elevator.l2());

        driver.povUp().whileTrue(elevator.move());
        driver.povRight().whileTrue(elevator.moveeng());
        driver.a().onTrue(new Command() {
            Timer timer = new Timer();

            @Override
            public void initialize() {
                timer.reset();
                timer.start();
            }

            @Override
            public void execute() {
                vis.setElevatorHeight(Inches.of(timer.get() * 30.0));
            }

            @Override
            public boolean isFinished() {
                return timer.hasElapsed(3.0);
            }
        });
        driver.b().onTrue(new Command() {
            Timer timer = new Timer();

            @Override
            public void initialize() {
                timer.reset();
                timer.start();
            }

            @Override
            public void execute() {
                vis.setElevatorHeight(Inches.of(Math.max(72.0 - timer.get() * 30.0, 0)));
            }

            @Override
            public boolean isFinished() {
                return timer.hasElapsed(3.0);
            }
        });

    }

    /**
     * Gets the user's selected autonomous command.
     *
     * @return Returns autonomous command selected.
     */
    public Command getAutonomousCommand() {
        return null;
    }

    /**
     * Update viz
     */
    public void updateViz() {
        vis.draw();
    }

    /** Start simulation */
    public void startSimulation() {
        if (driveSimulation != null) {
            SimulatedArena.getInstance().resetFieldForAuto();
        }
    }

    /**
     * Update simulation
     */
    public void updateSimulation() {
        if (driveSimulation != null) {
            SimulatedArena.getInstance().simulationPeriodic();
            Logger.recordOutput("FieldSimulation/Algae",
                SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
            Logger.recordOutput("FieldSimulation/Coral",
                SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
            vis.setActualPose(driveSimulation.getSimulatedDriveTrainPose());
        }
    }
}


