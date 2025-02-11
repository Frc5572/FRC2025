package frc.robot.autos;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve;

public class test {
    private Swerve swerve;
    private AutoFactory factory;

    public test(Swerve swerve) {
        this.swerve = swerve;
        this.factory = new AutoFactory(swerve::getPose, swerve::resetOdometry,
            swerve::followTrajectory, false, swerve);
    }

    public Command cmd() {
        Command testcmd = factory.trajectoryCmd("test");
        return testcmd;
    }
}
