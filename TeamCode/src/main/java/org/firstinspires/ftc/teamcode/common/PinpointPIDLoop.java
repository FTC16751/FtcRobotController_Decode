package org.firstinspires.ftc.teamcode.common;

/**
 * One axis of the Pinpoint point-to-point controller behind DriveUtil2026b.driveTo. DriveUtil
 * keeps three: x, y, and heading.
 *
 * PID with a filtered derivative, an integral cap, an asymmetric acceleration limit (ramping up
 * or reversing is rate-limited; braking is not), and a final clamp to +/-1. Time comes in as a
 * parameter, so this class is pure and unit-tested (PinpointPIDLoopTest).
 *
 * Quirks that are deliberate, and tested, because the autos were tuned against them:
 *   - The first call after construction or pidReset() only records the time and error and returns 0.
 *   - When the error is inside tolerance AND the previous output was already small, it treats the
 *     axis as settled: zeroes its history and returns 0.
 *
 * Moved out of DriveUtil2026b unchanged on 2026-09-07 so it could be tested.
 */
public class PinpointPIDLoop {
    private double previousError;
    private double previousTime;
    private double previousOutput;
    private double integralSum;
    private double filteredD;

    /** Largest contribution the integral term may make to the output, in motor power. */
    static final double I_MAX_OUTPUT = 0.2;
    /** Derivative filter: 0.85 old + 0.15 new. Slow on purpose; odometry is noisy. */
    static final double D_FILTER_KEEP = 0.85;

    /**
     * @param error       target minus current, in this axis's units (mm or radians)
     * @param pGain       power per unit of error
     * @param iGain       power per unit of accumulated error-seconds; 0 turns the term off
     * @param dGain       power per unit of error rate
     * @param accel       largest allowed output change per second when ramping up or reversing
     * @param currentTime seconds, from any monotonic clock
     * @param tolerance   error inside this counts as settled
     * @return motor power for this axis, -1 to 1
     */
    public double calculateAxisPID(double error, double pGain, double iGain, double dGain, double accel, double currentTime, double tolerance) {

        // First call initialization
        if (previousTime == 0.0) {
            previousTime = currentTime;
            previousError = error;
            return 0;
        }

        double cycleTime = currentTime - previousTime;
        if (cycleTime <= 1e-3) cycleTime = 1e-3;

        // Check if we're settled - partial reset to avoid pause
        if (Math.abs(error) <= tolerance && Math.abs(previousOutput) < 0.05) {
            previousOutput = 0;
            integralSum = 0;
            filteredD = 0;
            previousError = error;
            previousTime = currentTime;
            return 0;
        }

        // P term
        double p = error * pGain;

        // I term with anti-windup
        integralSum += error * cycleTime;
        if (iGain > 1e-9) {
            double maxIntegral = I_MAX_OUTPUT / iGain;
            integralSum = Math.max(-maxIntegral, Math.min(maxIntegral, integralSum));
        } else {
            integralSum = 0;
        }
        double i = iGain * integralSum;

        // D term on the error rate, filtered
        double rawD = (error - previousError) / cycleTime;
        filteredD = D_FILTER_KEEP * filteredD + (1.0 - D_FILTER_KEEP) * rawD;
        double d = dGain * filteredD;

        double output = p + i + d;

        // Asymmetric acceleration limiting
        double dV = cycleTime * accel;
        double outputChange = output - previousOutput;

        boolean isBraking = Math.abs(output) < Math.abs(previousOutput);
        boolean isReversing = (output * previousOutput) < 0;

        if (!isBraking || isReversing) {
            // Limit acceleration and direction changes
            if (outputChange > dV) {
                output = previousOutput + dV;
            } else if (outputChange < -dV) {
                output = previousOutput - dV;
            }
        }
        // Allow unlimited deceleration when braking (not reversing)

        // Final clamp to max power
        output = Math.max(-1.0, Math.min(1.0, output));

        previousOutput = output;
        previousError = error;
        previousTime = currentTime;

        return output;
    }

    /** Forget all history. The next call is a first call again (returns 0). */
    public void pidReset() {
        previousOutput = 0.0;
        previousError = 0.0;
        previousTime = 0.0;
        integralSum = 0.0;
        filteredD = 0.0;
    }
}
