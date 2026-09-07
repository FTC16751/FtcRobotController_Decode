package org.firstinspires.ftc.teamcode.common;

/** Test stand-in for VisionUtil: reports whatever the test sets. Shared by the aiming tests. */
class FakeAimTarget implements AimTarget {
    boolean visible = false;
    double distanceInches = 0.0;
    double angleXDeg = 0.0;

    FakeAimTarget see(double distanceInches, double angleXDeg) {
        this.visible = true;
        this.distanceInches = distanceInches;
        this.angleXDeg = angleXDeg;
        return this;
    }

    FakeAimTarget lose() {
        this.visible = false;
        return this;
    }

    @Override public boolean isTargetVisible()        { return visible; }
    @Override public double getDistanceToTagInches()  { return distanceInches; }
    @Override public double getTargetAngleX()         { return angleXDeg; }
}
