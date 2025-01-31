package frc.lib.util.viz;

import frc.robot.Constants;

/** Something that can be drawn to AdvantageScope. */
public interface Drawable {

    /** Draw to AdvantageScope */
    public void drawImpl();

    public default void draw() {
        if (Constants.shouldDrawStuff) {
            drawImpl();
        }
    }

    // TODO maybe layout stuff here?

}
