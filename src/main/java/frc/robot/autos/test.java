package frc.robot.autos;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve;

public class test {
    private Swerve swerve = new Swerve(null, null);
    private AutoFactory factory;
    Command testcmd = factory.trajectoryCmd("test");

    public test(Swerve swerve) {
        this.swerve = swerve;
        factory = new AutoFactory(swerve::getPose, swerve::resetOdometry, swerve::followTrajectory,
            false, swerve);
    }

    public Command cmd() {
        return testcmd;
    }
}
