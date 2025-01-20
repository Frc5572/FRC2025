package frc.lib.draw;

public class DrawingUtils {

    public static interface Drawable {
        public void draw();

        public void layout();
    }

    private static Drawable[] drawables = new Drawable[0];

    public static void addDrawable(Drawable drawable) {

    }

}
