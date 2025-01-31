package frc.lib.util.viz;

import frc.robot.Constants;

/** Something that can be drawn to AdvantageScope. */
public interface Drawable {

    /** Draw to AdvantageScope unconditionally. */
    public void drawImpl();

    /** Draw to AdvantageScope if drawing is enabled. */
    public default void draw() {
        if (Constants.shouldDrawStuff) {
            drawImpl();
        }
    }

    // TODO maybe layout stuff here?

}
