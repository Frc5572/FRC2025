
package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Meters;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.math.AllianceFlipUtil;
import frc.lib.math.Circle;
import frc.lib.math.Rectangle;
import frc.lib.math.SweptRectangles;
import frc.lib.util.swerve.SwerveModule;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotState;

/**
 * Swerve Subsystem
 */
public class Swerve extends SubsystemBase {
    public SwerveModule[] swerveMods;
    private final Field2d field = new Field2d();
    private double fieldOffset;
    private SwerveInputsAutoLogged inputs = new SwerveInputsAutoLogged();
    private SwerveIO swerveIO;
    private final RobotState state;

    /**
     * Swerve Subsystem
     */
    public Swerve(RobotState state, SwerveIO swerveIO) {
        super("Swerve");
        this.state = state;
        this.swerveIO = swerveIO;
        swerveMods = swerveIO.createModules();
        fieldOffset = getGyroYaw().getDegrees();

        swerveIO.updateInputs(inputs);

        state.init(getModulePositions(), getGyroYaw());
    }

    /**
     * Tele-Op Drive method
     *
     * @param translation The magnitude in XY
     * @param rotation The magnitude in rotation
     * @param fieldRelative Whether or not field relative
     * @param isOpenLoop Whether or not Open or Closed Loop
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative,
        boolean isOpenLoop) {
        ChassisSpeeds chassisSpeeds = fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(),
                rotation, getFieldRelativeHeading())
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);

        setModuleStates(chassisSpeeds);
    }

    /**
     * Set Swerve Module States
     *
     * @param desiredStates Array of desired states
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        Logger.recordOutput("/Swerve/DesiredStates", desiredStates);
        for (SwerveModule mod : swerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    /**
     * Sets swerve module states using Chassis Speeds.
     *
     * @param chassisSpeeds The desired Chassis Speeds
     */
    public void setModuleStates(ChassisSpeeds chassisSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(swerveModuleStates);
    }

    /**
     * Get current Chassis Speeds
     *
     * @return The current {@link ChassisSpeeds}
     */
    public ChassisSpeeds getChassisSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * Get Swerve Module States
     *
     * @return Array of Swerve Module States
     */
    @AutoLogOutput(key = "Swerve/ModuleStates")
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : swerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    /**
     * Get Swerve Module Positions
     *
     * @return Array of Swerve Module Positions
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : swerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    /**
     * Get Position on field from Odometry
     *
     * @return Pose2d on the field
     */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return state.getGlobalPoseEstimate();
    }

    /**
     * Set the position on the field with given Pose2d
     *
     * @param pose Pose2d to set
     */
    public void resetOdometry(Pose2d pose) {
        state.resetPose(pose, getModulePositions(), getGyroYaw());
        this.swerveIO.setPose(pose);
    }

    /**
     * Get Rotation of robot from odometry
     *
     * @return Heading of robot relative to the field as {@link Rotation2d}
     */
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    /**
     * Get Rotation from the gyro
     *
     * @return Current rotation/yaw of gyro as {@link Rotation2d}
     */
    public Rotation2d getGyroYaw() {
        double yaw = inputs.yaw;
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromRotations(-yaw)
            : Rotation2d.fromRotations(yaw);
    }

    /**
     * Get Field Relative Heading
     *
     * @return The current field relative heading in {@link Rotation2d}
     */
    public Rotation2d getFieldRelativeHeading() {
        return Rotation2d.fromDegrees(getGyroYaw().getDegrees() - fieldOffset);
    }

    /**
     * Resets the gyro field relative driving offset
     */
    public void resetFieldRelativeOffset() {
        fieldOffset = getGyroYaw().getDegrees() + 180;
    }

    @Override
    public void periodic() {

        swerveIO.updateInputs(inputs);
        for (var mod : swerveMods) {
            mod.periodic();
        }
        state.addSwerveObservation(getModulePositions(), getGyroYaw());
        Logger.processInputs("Swerve", inputs);
    }

    /**
     * Sets motors to 0 or inactive.
     */
    public void setMotorsZero() {
        System.out.println("Setting Zero!!!!!!");
        setModuleStates(new ChassisSpeeds(0, 0, 0));
    }

    /**
     * Make an X pattern with the wheels
     */
    public void wheelsIn() {
        swerveMods[0].setDesiredState(new SwerveModuleState(2, Rotation2d.fromDegrees(45)), false);
        swerveMods[1].setDesiredState(new SwerveModuleState(2, Rotation2d.fromDegrees(135)), false);
        swerveMods[2].setDesiredState(new SwerveModuleState(2, Rotation2d.fromDegrees(-45)), false);
        swerveMods[3].setDesiredState(new SwerveModuleState(2, Rotation2d.fromDegrees(-135)),
            false);
        this.setMotorsZero();
    }

    /**
     * Gets a list containing all 4 swerve module positions
     */
    public SwerveModulePosition[] getSwerveModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : swerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    /**
     * Determine whether or not to flight the auto path
     *
     * @return True if flip path to Red Alliance, False if Blue
     */
    public static boolean shouldFlipPath() {
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            return ally.get() == Alliance.Red;
        }
        return false;
    }

    /**
     * Creates a command for driving the swerve drive during tele-op
     *
     * @param controller The driver controller
     * @param fieldRelative Whether the movement is relative to the field or absolute
     * @param openLoop Open or closed loop system
     */
    public Command teleOpDrive(CommandXboxController controller, boolean fieldRelative,
        boolean openLoop) {
        return this.run(() -> {
            double speedMultiplier = 1;
            double yaxis = -controller.getLeftY() * speedMultiplier;
            double xaxis = -controller.getLeftX() * speedMultiplier;
            double raxis = -controller.getRightX() * speedMultiplier;
            /* Deadbands */
            yaxis = (Math.abs(yaxis) < Constants.STICK_DEADBAND) ? 0
                : (yaxis - Constants.STICK_DEADBAND) / (1.0 - Constants.STICK_DEADBAND);
            xaxis = (Math.abs(xaxis) < Constants.STICK_DEADBAND) ? 0
                : (xaxis - Constants.STICK_DEADBAND) / (1.0 - Constants.STICK_DEADBAND);
            xaxis *= xaxis * Math.signum(xaxis);
            yaxis *= yaxis * Math.signum(yaxis);
            raxis = (Math.abs(raxis) < Constants.STICK_DEADBAND) ? 0 : raxis;
            Translation2d translation =
                new Translation2d(yaxis, xaxis).times(Constants.Swerve.maxSpeed);
            double rotation = raxis * Constants.Swerve.maxAngularVelocity;
            this.drive(translation, rotation, fieldRelative, openLoop);
        });
    }

    /** Move to a given {@link Pose2d}. */
    public Command moveToPose(Supplier<Pose2d> pose2dSupplier, boolean flipForRed, double tol) {
        Swerve swerveCopy = this;

        HolonomicDriveController holonomicDriveController = new HolonomicDriveController(
            new PIDController(Constants.SwerveTransformPID.PID_XKP,
                Constants.SwerveTransformPID.PID_XKI, Constants.SwerveTransformPID.PID_XKD),
            new PIDController(Constants.SwerveTransformPID.PID_YKP,
                Constants.SwerveTransformPID.PID_YKI, Constants.SwerveTransformPID.PID_YKD),
            new ProfiledPIDController(Constants.SwerveTransformPID.PID_TKP,
                Constants.SwerveTransformPID.PID_TKI, Constants.SwerveTransformPID.PID_TKD,
                new TrapezoidProfile.Constraints(Constants.SwerveTransformPID.MAX_ANGULAR_VELOCITY,
                    Constants.SwerveTransformPID.MAX_ANGULAR_ACCELERATION)));
        holonomicDriveController.setTolerance(new Pose2d(tol, tol, Rotation2d.fromDegrees(1)));
        Command c = new Command() {

            private Pose2d pose2d;
            private Swerve swerve = swerveCopy;

            @Override
            public void initialize() {
                pose2d = pose2dSupplier.get();
                if (flipForRed) {
                    pose2d = AllianceFlipUtil.apply(pose2d);
                }
            }

            @Override
            public void execute() {
                ChassisSpeeds ctrlEffort = holonomicDriveController.calculate(swerve.getPose(),
                    pose2d, 0, pose2d.getRotation());
                swerve.setModuleStates(ctrlEffort);
            }

            @Override
            public void end(boolean interrupted) {
                swerve.setMotorsZero();
            }

            @Override
            public boolean isFinished() {
                return holonomicDriveController.atReference();
            }
        };
        c.addRequirements(this);
        return c;
    }

    /** Move to a given {@link Pose2d}. */
    public Command moveAndAvoidReef(Supplier<Pose2d> pose2dSupplier, boolean flipForRed,
        double tol) {
        Swerve swerveCopy = this;

        HolonomicDriveController holonomicDriveController = new HolonomicDriveController(
            new PIDController(Constants.SwerveTransformPID.PID_XKP,
                Constants.SwerveTransformPID.PID_XKI, Constants.SwerveTransformPID.PID_XKD),
            new PIDController(Constants.SwerveTransformPID.PID_YKP,
                Constants.SwerveTransformPID.PID_YKI, Constants.SwerveTransformPID.PID_YKD),
            new ProfiledPIDController(Constants.SwerveTransformPID.PID_TKP,
                Constants.SwerveTransformPID.PID_TKI, Constants.SwerveTransformPID.PID_TKD,
                new TrapezoidProfile.Constraints(Constants.SwerveTransformPID.MAX_ANGULAR_VELOCITY,
                    Constants.SwerveTransformPID.MAX_ANGULAR_ACCELERATION)));
        holonomicDriveController.setTolerance(new Pose2d(tol, tol, Rotation2d.fromDegrees(1)));
        Command c = new Command() {

            private Pose2d pose2d;
            private Swerve swerve = swerveCopy;

            @Override
            public void initialize() {
                pose2d = pose2dSupplier.get();
                if (flipForRed) {
                    pose2d = AllianceFlipUtil.apply(pose2d);
                }
                pose2d = RobotState.constrain(pose2d, (_newPose) -> {
                });
                targetBumpers.setPose(pose2d);
                if (DriverStation.getAlliance().get() == Alliance.Red) {
                    thisDriveCircle = driveCircleRed;
                } else {
                    thisDriveCircle = driveCircleBlue;
                }
            }

            private Circle thisDriveCircle;

            private final Rectangle targetBumpers = new Rectangle("moveAndAvoidReef/target",
                new Pose2d(), Constants.Swerve.bumperFront.in(Meters) * 2,
                Constants.Swerve.bumperRight.in(Meters) * 2);

            private SweptRectangles sweep = new SweptRectangles("moveAndAvoidReef/sweep",
                RobotState.robotBumperOutline, targetBumpers);

            private static final double driveCircleRadius =
                FieldConstants.Reef.circumscribedRadius.in(Meters)
                    + Math.hypot(Constants.Swerve.bumperFront.in(Meters),
                        Constants.Swerve.bumperRight.in(Meters));

            private final Circle driveCircleRed = new Circle("moveAndAvoidReef/driveCircleRed",
                new Translation2d(
                    FieldConstants.fieldLength.in(Meters) - FieldConstants.Reef.center.getX(),
                    FieldConstants.Reef.center.getY()),
                driveCircleRadius);

            private final Circle driveCircleBlue = new Circle("moveAndAvoidReef/driveCircleBlue",
                FieldConstants.Reef.center, driveCircleRadius);

            @Override
            public void execute() {
                targetBumpers.draw();
                driveCircleRed.draw();
                driveCircleBlue.draw();
                // sweep.draw();

                {
                    Translation2d diff =
                        swerve.getPose().getTranslation().minus(driveCircleBlue.getCenter());
                    Rotation2d angle = diff.getAngle();
                    double dist = diff.getNorm();
                    // TODO
                }

                driveAroundReef(driveCircleBlue, false);
            }

            private static final double radiusLookaheadAdj = driveCircleRadius
                / Constants.SwerveTransformPID.CIRCLE_REEF_LOOKAHEAD_ANGLE.getCos();

            private void driveAroundReef(Circle circle, boolean clockwise) {
                Rotation2d angle =
                    swerve.getPose().getTranslation().minus(circle.getCenter()).getAngle();
                Rotation2d targetAngle = clockwise
                    ? angle.minus(Constants.SwerveTransformPID.CIRCLE_REEF_LOOKAHEAD_ANGLE)
                    : angle.plus(Constants.SwerveTransformPID.CIRCLE_REEF_LOOKAHEAD_ANGLE);
                Pose2d targetPose = new Pose2d(
                    circle.getCenter().plus(new Translation2d(radiusLookaheadAdj, targetAngle)),
                    targetAngle.plus(Rotation2d.k180deg));
                driveToTarget(targetPose);
            }

            private void driveToTarget(Pose2d pose) {
                Logger.recordOutput("moveAndAvoidReef/intermediateTarget", pose);
                ChassisSpeeds ctrlEffort = holonomicDriveController.calculate(swerve.getPose(),
                    pose, 0, pose.getRotation());
                swerve.setModuleStates(ctrlEffort);
            }

            @Override
            public void end(boolean interrupted) {
                swerve.setMotorsZero();
            }

            @Override
            public boolean isFinished() {
                return holonomicDriveController.atReference();
            }
        };
        c.addRequirements(this);
        return c;
    }
}
