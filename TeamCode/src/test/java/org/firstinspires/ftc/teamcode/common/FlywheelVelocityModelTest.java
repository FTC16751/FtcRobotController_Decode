package org.firstinspires.ftc.teamcode.common;

import static org.junit.Assert.assertEquals;

import org.junit.Before;
import org.junit.Test;

public class FlywheelVelocityModelTest {

    private static final double[][] TABLE = {
            {30.0, 1200.0},
            {60.0, 1500.0},
            {90.0, 1800.0},
    };
    private static final double FALLBACK = 1350.0;

    private FlywheelVelocityModel model;
    private FakeAimTarget target;

    @Before
    public void setUp() {
        model = new FlywheelVelocityModel(TABLE, FALLBACK);
        target = new FakeAimTarget();
    }

    @Test
    public void usesFallbackBeforeTheGoalIsEverSeen() {
        assertEquals(FALLBACK, model.update(target.lose()), 0.0);
        assertEquals(FlywheelVelocityModel.SOURCE_LAST_KNOWN, model.getLastSource());
    }

    @Test
    public void looksUpTableRowsExactly() {
        assertEquals(1200.0, model.update(target.see(30.0, 0)), 0.0);
        assertEquals(1500.0, model.update(target.see(60.0, 0)), 0.0);
        assertEquals(1800.0, model.update(target.see(90.0, 0)), 0.0);
        assertEquals(FlywheelVelocityModel.SOURCE_VISION, model.getLastSource());
    }

    @Test
    public void interpolatesBetweenRows() {
        assertEquals(1350.0, model.update(target.see(45.0, 0)), 1e-9);
        assertEquals(1700.0, model.update(target.see(80.0, 0)), 1e-9);
    }

    @Test
    public void remembersLastGoodVelocityWhenTheGoalIsLost() {
        model.update(target.see(60.0, 0));               // 1500
        assertEquals(1500.0, model.update(target.lose()), 0.0);
        assertEquals(FlywheelVelocityModel.SOURCE_LAST_KNOWN, model.getLastSource());
        assertEquals(1500.0, model.getLastKnownGoodVelocity(), 0.0);
    }

    @Test
    public void seeingTheGoalAgainReplacesTheRememberedValue() {
        model.update(target.see(60.0, 0));               // 1500
        model.update(target.lose());
        assertEquals(1200.0, model.update(target.see(30.0, 0)), 0.0);
        assertEquals(1200.0, model.update(target.lose()), 0.0);
    }

    @Test
    public void velocityForDistanceDoesNotTouchTheRememberedValue() {
        model.update(target.see(60.0, 0));               // remember 1500
        assertEquals(1800.0, model.velocityForDistance(90.0), 0.0);
        assertEquals(1500.0, model.getLastKnownGoodVelocity(), 0.0);
    }

    @Test
    public void rowOrderDoesNotMatter() {
        FlywheelVelocityModel shuffled = new FlywheelVelocityModel(
                new double[][]{{90.0, 1800.0}, {30.0, 1200.0}, {60.0, 1500.0}}, FALLBACK);
        assertEquals(1350.0, shuffled.velocityForDistance(45.0), 1e-9);
    }
}
