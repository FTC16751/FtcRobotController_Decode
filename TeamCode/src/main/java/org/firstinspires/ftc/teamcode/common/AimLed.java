package org.firstinspires.ftc.teamcode.common;

/**
 * Drives the status LED from where the goal is: green when lined up, one color when the goal is
 * to the right, another when it is to the left, off when it is not visible.
 *
 * Every robot did this in its own updateLedStatus() before 2026-09-07 with slightly different
 * tolerances and colors; those are now settings the team passes in.
 */
public class AimLed {

    /** Which LedUtil.Color value to show for each situation. Defaults are what most robots used. */
    public static class Colors {
        public double onTarget = LedUtil.Color.GREEN;
        public double goalToRight = LedUtil.Color.YELLOW;   // positive angle
        public double goalToLeft = LedUtil.Color.BLUE;      // negative angle
        public double noTarget = LedUtil.Color.OFF;

        public Colors onTarget(double c)    { this.onTarget = c; return this; }
        public Colors goalToRight(double c) { this.goalToRight = c; return this; }
        public Colors goalToLeft(double c)  { this.goalToLeft = c; return this; }
        public Colors noTarget(double c)    { this.noTarget = c; return this; }
    }

    private final LedUtil led;        // may be null on a robot with no LED
    private final AimTarget target;
    private final double toleranceDeg;
    private final Colors colors;

    public AimLed(LedUtil led, AimTarget target, double toleranceDeg, Colors colors) {
        this.led = led;
        this.target = target;
        this.toleranceDeg = toleranceDeg;
        this.colors = colors != null ? colors : new Colors();
    }

    /** The color the LED should show right now. Pure function of the target; unit-testable. */
    public double chooseColor() {
        if (target == null || !target.isTargetVisible()) {
            return colors.noTarget;
        }
        double angle = target.getTargetAngleX();
        if (Math.abs(angle) <= toleranceDeg) return colors.onTarget;
        return angle > 0 ? colors.goalToRight : colors.goalToLeft;
    }

    /** Call every loop. Does nothing on a robot without an LED. */
    public void update() {
        if (led == null) return;
        led.setColor(chooseColor());
    }
}
