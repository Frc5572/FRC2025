package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * LEDs subsystem
 */
public class LEDs extends SubsystemBase {
    private final AddressableLEDBufferView chosenLEDside;


    LEDPattern rainbow = LEDPattern.rainbow(255, 128);
    LEDPattern rainbowPattern = rainbow.scrollAtRelativeSpeed(Percent.per(Second).of(100));

    @Override
    public void periodic() {}

    /**
     * constructor
     */
    public LEDs(AddressableLEDBuffer passedBuffer, int start, int end) {
        this(passedBuffer, start, end, false);
    }

    public LEDs(AddressableLEDBuffer passedBuffer, int start, int end, boolean reversed) {
        chosenLEDside = reversed ? passedBuffer.createView(start, end).reversed()
            : passedBuffer.createView(start, end);
    }

    /**
     * Blink LEDs
     *
     * @param mainColor color to blink (on and off)
     *
     * @return Command to blink leds
     */
    public Command blinkLEDs(Color mainColor) {
        LEDPattern colorToPattern = LEDPattern.solid(mainColor);
        LEDPattern blinkPattern = colorToPattern.blink(Seconds.of(.5));
        return run(() -> blinkPattern.applyTo(chosenLEDside)).ignoringDisable(true);
    }

    /**
     * Blink LEDs for a certain timeout
     *
     * @param mainColor color to blink (on and off)
     * @param timeout Number of seconds to blink
     *
     * @return Command to blink leds
     */
    public Command blinkLEDs(Color mainColor, double timeout) {
        return blinkLEDs(mainColor).withTimeout(timeout);
    }

    /**
     * Set LEDs to solid color
     *
     * @param color color to set the leds to solidly
     *
     * @return Command to set leds to a solid color
     */
    public Command setLEDsSolid(Color color) {
        LEDPattern solidPattern = LEDPattern.solid(color);
        return run(() -> solidPattern.applyTo(chosenLEDside)).ignoringDisable(true);
    }

    /**
     * Set LEDs to color gradient
     *
     * @param color first color for gradient
     * @param color2 second color for gradient
     *
     * @return sets color gradient
     */
    public Command setLEDsGradient(Color color, Color color2) {
        LEDPattern gradientPattern = LEDPattern.gradient(GradientType.kContinuous, color, color2);
        return run(() -> gradientPattern.applyTo(chosenLEDside)).ignoringDisable(true);
    }

    /**
     * Set LEDs to breathe
     *
     * @param color Color to set leds to
     *
     * @return leds breathe command
     */
    public Command setLEDsBreathe(Color color) {
        LEDPattern base = LEDPattern.solid(color);
        LEDPattern breathe = base.breathe(Seconds.of(2));
        return run(() -> breathe.applyTo(chosenLEDside)).ignoringDisable(true);
    }


    /**
     * Set LEDs to police pattern/color
     *
     *
     *
     *
     */
    public Command setPoliceLeds(Boolean isRight) {
        if (isRight) {
            LEDPattern notbase2 = LEDPattern.solid(Color.kRed);
            LEDPattern notbase3 = LEDPattern.solid(Color.kBlue);
            LEDPattern blink = notbase2.blink(Seconds.of(0.5)).overlayOn(notbase3);
            return run(() -> {
                blink.applyTo(chosenLEDside);
            }).ignoringDisable(true);

        } else {
            LEDPattern notbase2 = LEDPattern.solid(Color.kBlue);
            LEDPattern notbase3 = LEDPattern.solid(Color.kRed);
            LEDPattern blink = notbase2.blink(Seconds.of(0.5)).overlayOn(notbase3);
            return run(() -> {
                blink.applyTo(chosenLEDside);
            }).ignoringDisable(true);
        }


    }

    /**
     * Sets specified LED side to Rainbow even when disabled
     *
     *
     *
     *
     */
    public Command setRainbow() {
        return run(() -> {
            rainbowPattern.applyTo(chosenLEDside);
        }).ignoringDisable(true);

    }


}
