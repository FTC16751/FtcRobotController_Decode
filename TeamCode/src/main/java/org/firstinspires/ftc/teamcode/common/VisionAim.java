package org.firstinspires.ftc.teamcode.common;

/**
 * Turn toward the goal: the proportional "snap to target" every TeleOp and the P3 Queue autos
 * carried as three inline lines. Zero inside the tolerance, gain times angle outside it.
 *
 * Positive angle means the goal is to the right and the returned power turns the robot right,
 * matching how every OpMode used it before 2026-09-07. Gains and tolerances stay in the OpMode
 * (or the team's Constants); this class only does the arithmetic.
 */
public final class VisionAim {

    private VisionAim() {}

    /** True when the goal is within toleranceDeg of straight ahead. */
    public static boolean onTarget(double angleXDeg, double toleranceDeg) {
        return Math.abs(angleXDeg) <= toleranceDeg;
    }

    /** Turn power for a known angle: 0 inside the tolerance, kp * angle outside it. Not clamped. */
    public static double turnPower(double angleXDeg, double kp, double toleranceDeg) {
        return onTarget(angleXDeg, toleranceDeg) ? 0.0 : kp * angleXDeg;
    }

    /** Same, read straight from vision. 0 when the goal is not visible. */
    public static double turnPower(AimTarget target, double kp, double toleranceDeg) {
        if (target == null || !target.isTargetVisible()) return 0.0;
        return turnPower(target.getTargetAngleX(), kp, toleranceDeg);
    }
}
