package frc.robot.subsystems.elevator_coral;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants;

public class ElevatorCoralReal implements ElevatorCoralIO {

    private final SparkMax feederMotor =
        new SparkMax(Constants.Motors.AlgaeMotors.FEEDER_MOTOR_ID, MotorType.kBrushless);

    @Override
    public void setFeederMotorSpeed(double speed) {
        feederMotor.set(speed);
    }
}
