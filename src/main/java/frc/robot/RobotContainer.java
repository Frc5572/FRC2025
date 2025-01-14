package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot.RobotRunType;
import frc.robot.subsystems.swerve.drive.Swerve;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final CommandXboxController driver = new CommandXboxController(Constants.driverID);
    private final CommandXboxController operator = new CommandXboxController(Constants.operatorID);

    // Initialize AutoChooser Sendable
    private final SendableChooser<String> autoChooser = new SendableChooser<>();

    /* Subsystems */
    private Swerve s_Swerve;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
   public RobotContainer(RobotRunType runtimeType) {
        // if (runtimeType == RobotRunType.kSimulation) {
        //     SimulatedArena.overrideSimulationTimings(Units.Seconds.of(0.02), 5);
        // }

        switch (runtimeType) {
            case kReal:
                s_Swerve = new Swerve(new SwerveBoron(), SwerveModuleTalonAngle::new,
                    SwerveModuleTalonDrive::new);
                break;
            // case kSimulation:
            //     driveSimulation =
            //         new SwerveDriveSimulation(Constants.Swerve.getMapleConfig(),
            //             new Pose2d(3, 3, Rotation2d.kZero));
            //     SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
            //     s_Swerve = new Swerve(new SwerveSim(driveSimulation),
            //         (i, config) -> {
            //             var sim = new SwerveModuleSim(config, driveSimulation.getModules()[i]);
            //             return Pair.of(sim, sim);
            //         });
            //     break;
            default:
                // s_Swerve = new Swerve(new SwerveIO();



    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {}

    /**
     * Gets the user's selected autonomous command.
     *
     * @return Returns autonomous command selected.
     */
    public Command getAutonomousCommand() {
        Command autocommand;
        String stuff = autoChooser.getSelected();
        switch (stuff) {
            case "wait":
                autocommand = new WaitCommand(1.0);
                break;
            default:
                autocommand = new InstantCommand();
        }
        return autocommand;
    }
}
