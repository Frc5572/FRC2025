package frc.robot;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.sim.SimulatedArena;
import frc.lib.viz.RobotViz;
import frc.robot.Robot.RobotRunType;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveIO;
import frc.robot.subsystems.swerve.SwerveReal;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    /* Controllers */
    public final CommandXboxController driver = new CommandXboxController(Constants.driverId);

    private SwerveDriveSimulation driveSimulation;

    /* Subsystems */
    private Swerve s_Swerve;

    /** Viz */
    private final RobotViz viz;

    /** Simulation */
    private final SimulatedArena arena;

    /**
     */
    public RobotContainer(RobotRunType runtimeType) {
        if (runtimeType == RobotRunType.kSimulation) {
            arena = new SimulatedArena();
        } else {
            arena = null;
        }
        switch (runtimeType) {
            case kReal:
                viz = new RobotViz("Viz", null);
                s_Swerve = new Swerve(new SwerveReal(), viz);
                break;
            // case kSimulation:
            // SimulatedRobot robot = arena.newRobot();
            // s_Swerve = new Swerve(new SwerveSim(robot), viz);
            // break;
            default:
                viz = new RobotViz("Viz", null);
                s_Swerve = new Swerve(new SwerveIO() {}, viz);
        }


        s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, driver,
            Constants.Swerve.isFieldRelative, Constants.Swerve.isOpenLoop));

        configureButtonBindings(runtimeType);
    }

    /**
     * Use this method to vol your button->command mappings. Buttons can be created by instantiating
     * a {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or
     * {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings(RobotRunType runtimeType) {}

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

    }

    /** Start simulation */
    public void startSimulation() {
        if (driveSimulation != null) {
        }
    }

    /**
     * Update simulation
     */
    public void updateSimulation() {

    }

}
