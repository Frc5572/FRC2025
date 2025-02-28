package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LEDs.LEDsLeft;
import frc.robot.subsystems.LEDs.LEDsRight;

public class policeLights extends Command {
    private LEDs.LEDsLeft ledsLeft;
    private LEDs.LEDsRight ledsRight;

    policeLights() {
        this.ledsLeft = new LEDsLeft();
        this.ledsRight = new LEDsRight();
    }

    public Command setPoliceLeds() {
        LEDPattern notbase2 = LEDPattern.solid(Color.kBlue);
        LEDPattern notbase3 = LEDPattern.solid(Color.kRed);
        LEDPattern blink2 = notbase3.blink(Seconds.of(0.5)).overlayOn(notbase2);
        LEDPattern blink = notbase2.blink(Seconds.of(0.5)).overlayOn(notbase3);
        return Commands.run(() -> {
            blink.applyTo(ledsRight.leds_right);
            blink2.applyTo(ledsLeft.leds_left);
        }).ignoringDisable(true);

    }
}
