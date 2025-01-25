package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDs extends SubsystemBase {
    private final AddressableLED leds;
    private final AddressableLEDBuffer buffer;

    LEDPattern rainbow = LEDPattern.rainbow(100, 255);

    public LEDs() {
        leds = new AddressableLED(Constants.LEDs.LED_PORT);
        buffer = new AddressableLEDBuffer(Constants.LEDs.LED_LENGTH);
        leds.setLength(Constants.LEDs.LED_LENGTH);
        leds.start();

    }

    public void blinkLEDs(LEDPattern mainColor) {
        LEDPattern blinkPattern = mainColor.blink(Time.ofBaseUnits(.5, Second));
        blinkPattern.applyTo(buffer);
        leds.setData(buffer);
    }

    public void setLEDsSolid(Color color) {
        LEDPattern solidPattern = LEDPattern.solid(color);
        solidPattern.applyTo(buffer);
        leds.setData(buffer);
    }

    public void setLEDsGradient(Color color, Color color2) {
        LEDPattern gradientPattern = LEDPattern.gradient(GradientType.kContinuous, color, color2);
        gradientPattern.applyTo(buffer);
        leds.setData(buffer);
    }
}
