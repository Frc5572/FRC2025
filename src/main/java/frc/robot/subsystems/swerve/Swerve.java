
package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Rotation;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.LoggedTracer;
import frc.lib.util.swerve.SwerveModule;
import frc.robot.Constants;
import frc.robot.Robot.RobotRunType;
import frc.robot.RobotContainer;
import frc.robot.RobotState;

/**
 * Swerve Subsystem
 */
public class Swerve extends SubsystemBase {
    public SwerveModule[] swerveMods;
    private final Field2d field = new Field2d();
    private double fieldOffset;
    private SwerveInputsAutoLogged inputsSwerve = new SwerveInputsAutoLogged();
    private SwerveIO swerveIO;
    private GyroIO gyroIO;
    private GyroInputsAutoLogged inputsGyro = new GyroInputsAutoLogged();

    public final RobotState state;
    private double setSpeedMultiplier = 1.0;
    private final HolonomicDriveController holonomicDriveController = new HolonomicDriveController(
        new PIDController(Constants.SwerveTransformPID.PID_XKP,
            Constants.SwerveTransformPID.PID_XKI, Constants.SwerveTransformPID.PID_XKD),
        new PIDController(Constants.SwerveTransformPID.PID_XKP,
            Constants.SwerveTransformPID.PID_XKI, Constants.SwerveTransformPID.PID_XKD),
        new ProfiledPIDController(Constants.SwerveTransformPID.PID_TKP,
            Constants.SwerveTransformPID.PID_TKI, Constants.SwerveTransformPID.PID_TKD,
            new TrapezoidProfile.Constraints(Constants.SwerveTransformPID.MAX_ANGULAR_VELOCITY,
                Constants.SwerveTransformPID.MAX_ANGULAR_ACCELERATION)));
    public boolean initGyroBool = true;
    public Trigger initGyro = new Trigger(() -> initGyroBool);


    /**
     * Swerve Subsystem
     */
    public Swerve(RobotState state, SwerveIO swerveIO, GyroIO gyroIO) {
        super("Swerve");
        this.state = state;
        this.swerveIO = swerveIO;
        this.gyroIO = gyroIO;
        swerveMods = swerveIO.createModules();
        fieldOffset = getGyroYaw().getDegrees();

        swerveIO.updateInputs(inputsSwerve);
        gyroIO.updateInputs(inputsGyro);
        state.init(getModulePositions(), getGyroYaw());
        SmartDashboard.putData(Constants.DashboardValues.field2d, field);

        if (RobotContainer.runType == RobotRunType.kSimulation) {
            holonomicDriveController.getXController().setP(7.0);
            holonomicDriveController.getYController().setP(7.0);
        }
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
        Logger.recordOutput("Swerve/Velocity",
            Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond));
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
        return Rotation2d.fromRotations(inputsGyro.yaw.in(Rotation));
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
        fieldOffset = getGyroYaw().getDegrees();
    }

    /**
     * Set Field Relative Offset based on Pose
     */
    public void resetFieldRelativeOffsetBasedOnPose() {
        double redSideflip = shouldFlipPath() ? 180.0 : 0.0;
        fieldOffset =
            (getGyroYaw().getDegrees() - state.getGlobalPoseEstimate().getRotation().getDegrees()
                + 180 + redSideflip) % 360 - 180;
        initGyroBool = false;
    }

    @Override
    public void periodic() {


        swerveIO.updateInputs(inputsSwerve);
        gyroIO.updateInputs(inputsGyro);

        for (var mod : swerveMods) {
            mod.periodic();
        }
        state.addSwerveObservation(getModulePositions(), getGyroYaw());
        Logger.processInputs("Drive/Swerve", inputsSwerve);
        Logger.processInputs("Drive/gyro", inputsGyro);
        field.setRobotPose(getPose());
        SmartDashboard.putNumber("SpeedMultiplier", setSpeedMultiplier);
        LoggedTracer.record("Swerve");
        Logger.recordOutput("Swerve/Field Offset", fieldOffset);
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

    public void setSpeedMultiplier(double multiplier) {
        setSpeedMultiplier = multiplier;
    }

    public double getSpeedMultiplier() {
        return setSpeedMultiplier;
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
            double yaxis = -controller.getLeftY();
            double xaxis = -controller.getLeftX();
            double raxis = -controller.getRightX();
            /* Deadbands */
            yaxis = MathUtil.applyDeadband(yaxis, 0.1);
            xaxis = MathUtil.applyDeadband(xaxis, 0.1);
            xaxis *= xaxis * Math.signum(xaxis);
            yaxis *= yaxis * Math.signum(yaxis);
            raxis = (Math.abs(raxis) < Constants.STICK_DEADBAND) ? 0 : raxis;
            Translation2d translation = new Translation2d(yaxis, xaxis)
                .times(Constants.Swerve.maxSpeed).times(setSpeedMultiplier);
            double rotation = raxis * Constants.Swerve.maxAngularVelocity * setSpeedMultiplier;
            this.drive(translation, rotation, fieldRelative, openLoop);
        });
    }



    public Command stop() {
        return this.runOnce(this::setMotorsZero);
    }

    /**
     * Follow Choreo Trajectory
     *
     * @param sample Swerve Sample
     */
    public void followTrajectory(SwerveSample sample) {
        // Get the current pose of the robot
        Pose2d pose = getPose();

        // Generate the next speeds for the robot
        ChassisSpeeds speeds = new ChassisSpeeds(
            sample.vx + holonomicDriveController.getXController().calculate(pose.getX(), sample.x),
            sample.vy + holonomicDriveController.getYController().calculate(pose.getY(), sample.y),
            sample.omega + holonomicDriveController.getThetaController()
                .calculate(pose.getRotation().getRadians(), sample.heading));

        // Apply the generated speeds
        setModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(speeds,
            state.getGlobalPoseEstimate().getRotation()));
    }

    /**
     * Move to a Pose2d
     *
     * @param pose Desired Pose2d
     */
    public void moveToPose(Pose2d pose, double maxSpeed, double maxAcceleration) {
        if (Constants.shouldDrawStuff) {
            Logger.recordOutput("Swerve/moveToPoseTarget", pose);
        }
        ChassisSpeeds ctrlEffort = holonomicDriveController.calculate(state.getGlobalPoseEstimate(),
            pose, 0, pose.getRotation());
        double speed = Math.hypot(ctrlEffort.vxMetersPerSecond, ctrlEffort.vyMetersPerSecond);
        if (speed > maxSpeed) {
            double mul = maxSpeed / speed;
            ctrlEffort.vxMetersPerSecond *= mul;
            ctrlEffort.vyMetersPerSecond *= mul;
        }
        setModuleStates(ctrlEffort);
    }

    /**
     * Move to a Pose2d
     *
     * @param pose Desired Pose2d
     */
    public void moveToPose(Pose2d pose, double maxVelocity) {
        moveToPose(pose, maxVelocity, Constants.SwerveTransformPID.MAX_ACCELERATION);
    }

    /**
     * Move to a Pose2d
     *
     * @param pose Desired Pose2d
     */
    public void moveToPose(Pose2d pose) {
        moveToPose(pose, Constants.SwerveTransformPID.MAX_VELOCITY,
            Constants.SwerveTransformPID.MAX_ACCELERATION);
    }

}
