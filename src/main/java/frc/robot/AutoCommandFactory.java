package frc.robot;

import choreo.auto.AutoFactory;
import frc.robot.subsystems.coral.CoralScoring;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.swerve.Swerve;

public class AutoCommandFactory {

    AutoFactory autoFactory;
    Swerve swerve;
    Elevator elevator;
    CoralScoring coral;

    public AutoCommandFactory(AutoFactory autoFactory, Swerve swerve, Elevator elevator,
        CoralScoring coral) {
        this.autoFactory = autoFactory;
        this.swerve = swerve;
        this.elevator = elevator;
        this.coral = coral;
    }

}
