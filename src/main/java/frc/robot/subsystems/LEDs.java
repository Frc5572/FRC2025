package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * LEDs subsystem
 */
public class LEDs extends SubsystemBase {
    private final AddressableLED leds;
    private final AddressableLEDBuffer buffer;

    LEDPattern rainbow = LEDPattern.rainbow(100, 255);

    @Override
    public void periodic() {
        leds.setData(buffer);
    }

    /**
     * constructor
     */
    public LEDs() {
        leds = new AddressableLED(Constants.LEDs.LED_PORT);
        buffer = new AddressableLEDBuffer(Constants.LEDs.LED_LENGTH);
        leds.setLength(Constants.LEDs.LED_LENGTH);
        leds.start();

    }

    public class LEDsRight extends SubsystemBase {
        public final AddressableLEDBufferView leds_right;

        public LEDsRight() {
            leds_right = buffer.createView(60, 119);

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
            return run(() -> blinkPattern.applyTo(leds_right)).ignoringDisable(true);
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
            return run(() -> solidPattern.applyTo(leds_right)).ignoringDisable(true);
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
            return run(() -> breathe.applyTo(leds_right)).ignoringDisable(true);
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
            LEDPattern gradientPattern =
                LEDPattern.gradient(GradientType.kContinuous, color, color2);
            return run(() -> gradientPattern.applyTo(leds_right)).ignoringDisable(true);
        }
    }

    public class LEDsLeft extends SubsystemBase {
        public final AddressableLEDBufferView leds_left;

        public LEDsLeft() {
            leds_left = buffer.createView(0, 59);

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
            return run(() -> blinkPattern.applyTo(leds_left)).ignoringDisable(true);
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
            return run(() -> solidPattern.applyTo(leds_left)).ignoringDisable(true);
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
            return run(() -> breathe.applyTo(leds_left)).ignoringDisable(true);
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
            LEDPattern gradientPattern =
                LEDPattern.gradient(GradientType.kContinuous, color, color2);
            return run(() -> gradientPattern.applyTo(leds_left)).ignoringDisable(true);
        }
    }
}
