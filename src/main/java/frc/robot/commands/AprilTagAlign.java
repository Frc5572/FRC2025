package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.math.FilteredPIDController;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Vision;

public class AprilTagAlign extends Command {

    private final Swerve swerve;
    private final Vision vision;
    private final DoubleSupplier targetYawSupplier;

    private final FilteredPIDController controller = new FilteredPIDController(0.03, 0, 0.0, 0.1);

    public AprilTagAlign(Swerve swerve, Vision vision, DoubleSupplier targetYawSupplier) {
        this.swerve = swerve;
        this.vision = vision;
        this.targetYawSupplier = targetYawSupplier;
        addRequirements(swerve);
    }

    private double targetYaw;

    @Override
    public void initialize() {
        targetYaw = targetYawSupplier.getAsDouble();
        controller.setSetpoint(targetYaw);
    }

    @Override
    public void execute() {
        double output = controller.calculate(vision.getClosestTagYaw());
        swerve.setModuleStates(new ChassisSpeeds(0, output, 0));
    }

}
