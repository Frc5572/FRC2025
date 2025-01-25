package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDs extends SubsystemBase {
    private final AddressableLED leds;
    private final AddressableLEDBuffer buffer;

    LEDPattern rainbow = LEDPattern.rainbow(100, 255);

    @Override
    public void periodic() {
        leds.setData(buffer);
    }

    public LEDs() {
        leds = new AddressableLED(Constants.LEDs.LED_PORT);
        buffer = new AddressableLEDBuffer(Constants.LEDs.LED_LENGTH);
        leds.setLength(Constants.LEDs.LED_LENGTH);
        leds.start();

    }

    public Command blinkLEDs(LEDPattern mainColor) {
        LEDPattern blinkPattern = mainColor.blink(Seconds.of(1));
        return run(() -> blinkPattern.applyTo(buffer));
    }

    public Command setLEDsSolid(Color color) {
        LEDPattern solidPattern = LEDPattern.solid(color);
        return run(() -> solidPattern.applyTo(buffer));
    }

    public Command setLEDsGradient(Color color, Color color2) {
        LEDPattern gradientPattern = LEDPattern.gradient(GradientType.kContinuous, color, color2);
        return run(() -> gradientPattern.applyTo(buffer));
    }

    public Command setLEDsBreathe(Color color) {
        LEDPattern base = LEDPattern.solid(color);
        LEDPattern breathe = base.breathe(Seconds.of(2));
        return run(() -> breathe.applyTo(buffer));
    }
}
