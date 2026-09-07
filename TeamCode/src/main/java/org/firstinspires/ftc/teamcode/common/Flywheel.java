package org.firstinspires.ftc.teamcode.common;

/**
 * The one thing LaunchController needs from a flywheel, whatever motor class a team wrote.
 *
 * Each team adapts its own launcher in a few lines, usually as a lambda or a tiny inner class in
 * the robot class:
 * <pre>
 *   Flywheel wheel = new Flywheel() {
 *       public void setVelocity(double v) { launcher.setShooterMotorVelocity(v); }
 *       public double getVelocity()      { return launcher.getShooterMotorVelocity(); }
 *   };
 * </pre>
 * Velocity units are whatever the team's launcher uses (ticks per second on every robot today);
 * LaunchController only compares the two numbers to each other.
 */
public interface Flywheel {
    /** Command the flywheel to hold this velocity. 0 stops it. */
    void setVelocity(double velocity);

    /** Current measured velocity in the same units as setVelocity. */
    double getVelocity();
}
