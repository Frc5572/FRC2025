package frc.robot.subsystems.elevator_coral;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants;

/*
 * Elevator Coral real class
 */
public class ElevatorCoralReal implements ElevatorCoralIO { // elevator Real class

    private final SparkMax feederMotor = // Coral motor
        new SparkMax(Constants.Motors.AlgaeMotors.FEEDER_MOTOR_ID, MotorType.kBrushless);

    @Override
    public void setFeederMotorSpeed(double speed) { // set hardware speed
        feederMotor.set(speed);
    }
}
