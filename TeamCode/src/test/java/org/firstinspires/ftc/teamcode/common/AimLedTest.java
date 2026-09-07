package org.firstinspires.ftc.teamcode.common;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class AimLedTest {

    private static final double TOL = 2.0;

    private AimLed led(FakeAimTarget t) {
        return new AimLed(null, t, TOL, new AimLed.Colors());   // no physical LED in tests
    }

    @Test
    public void offWhenTheGoalIsNotVisible() {
        assertEquals(LedUtil.Color.OFF, led(new FakeAimTarget().lose()).chooseColor(), 0.0);
    }

    @Test
    public void greenWhenLinedUp() {
        assertEquals(LedUtil.Color.GREEN, led(new FakeAimTarget().see(60, 0.0)).chooseColor(), 0.0);
        assertEquals(LedUtil.Color.GREEN, led(new FakeAimTarget().see(60, 1.9)).chooseColor(), 0.0);
        assertEquals(LedUtil.Color.GREEN, led(new FakeAimTarget().see(60, -1.9)).chooseColor(), 0.0);
    }

    @Test
    public void exactlyAtToleranceCountsAsLinedUp() {
        assertEquals(LedUtil.Color.GREEN, led(new FakeAimTarget().see(60, TOL)).chooseColor(), 0.0);
        assertEquals(LedUtil.Color.GREEN, led(new FakeAimTarget().see(60, -TOL)).chooseColor(), 0.0);
    }

    @Test
    public void yellowWhenTheGoalIsToTheRight() {
        assertEquals(LedUtil.Color.YELLOW, led(new FakeAimTarget().see(60, 5.0)).chooseColor(), 0.0);
    }

    @Test
    public void blueWhenTheGoalIsToTheLeft() {
        assertEquals(LedUtil.Color.BLUE, led(new FakeAimTarget().see(60, -5.0)).chooseColor(), 0.0);
    }

    @Test
    public void teamCanChooseItsOwnColors() {
        AimLed skylineStyle = new AimLed(null, new FakeAimTarget().see(60, 5.0), TOL,
                new AimLed.Colors().goalToRight(LedUtil.Color.ORANGE));
        assertEquals(LedUtil.Color.ORANGE, skylineStyle.chooseColor(), 0.0);
    }

    @Test
    public void widerToleranceIsHonored() {
        AimLed p3Style = new AimLed(null, new FakeAimTarget().see(60, 3.5), 4.0, null);
        assertEquals(LedUtil.Color.GREEN, p3Style.chooseColor(), 0.0);
    }

    @Test
    public void updateWithoutAnLedDoesNotThrow() {
        led(new FakeAimTarget().see(60, 5.0)).update();
    }
}
