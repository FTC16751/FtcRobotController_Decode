package org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot;

import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.ArrayList;
import java.util.List;

/**
 * ShotSequenceController - Simplified 3-Shot Manager
 * ==================================================
 *
 * Two modes:
 * 1. COLOR MODE: Fires balls matching the motif pattern (GPP, PGP, PPG)
 *    - Checks left/right sensors
 *    - Rotates spinner ONCE if needed to access center ball
 *    - Resets spinner at end
 *
 * 2. PURGE MODE: Fast blind fire for clearing all balls
 *    - Fire both flippers
 *    - Rotate spinner
 *    - Fire left flipper
 *    - Reset spinner
 *
 * SPINNER MECHANICS:
 * When spinner rotates LEFT, the physical ball positions shift:
 *   - Center ball → Left position
 *   - Right ball → Center position
 *   - Left ball → Right position
 *
 * @author GearGirls Team
 * @version 4.2 - Fixed spinner reset causing infinite loop
 */
public class ShotSequenceControllerV2 {

    private enum State {
        IDLE,
        FIND_AND_FIRE,      // Find ball and fire it
        WAIT_FLIPPER,       // Wait for flipper to finish
        ROTATE_SPINNER,     // Rotate to access center ball
        WAIT_SPINNER,       // Wait for spinner
        RESET_SPINNER,      // Return spinner to home
        WAIT_RESET_SPINNER, // Wait for reset to complete
        DONE
    }

    private final GGRobot2 robot;
    private State state = State.IDLE;
    private List<IntakeSensorFusion002.ArtifactColor> shotSequence = new ArrayList<>();
    private int currentShot = 0;
    private boolean spinnerWasRotated = false;
    private boolean purgeMode = false;
    private final ElapsedTime timer = new ElapsedTime();

    // Settings
    private double launcherMinVelocity = 1850.0;
    private static final double SPINNER_TIMEOUT = 0.8;
    private static final double FLIPPER_TIMEOUT = 1.0;
    private static final double LAUNCHER_TIMEOUT = 2.0;

    public ShotSequenceControllerV2(GGRobot2 robot) {
        this.robot = robot;
    }

    //========================================
    // PUBLIC API
    //========================================

    /**
     * Start a new sequence with color matching
     */
    public void start(String motif) {
        reset();
        this.shotSequence = parseMotif(motif == null ? "PPG" : motif);
        this.purgeMode = false;
        this.state = State.FIND_AND_FIRE;
        timer.reset();
    }

    /**
     * Purge all balls without color matching
     */
    public void purge() {
        reset();
        this.purgeMode = true;
        this.state = State.FIND_AND_FIRE;
        timer.reset();
    }

    /**
     * Stop and reset
     */
    public void reset() {
        state = State.IDLE;
        currentShot = 0;
        spinnerWasRotated = false;
        purgeMode = false;
        shotSequence.clear();
    }

    /**
     * Call every loop
     */
    public void update() {
        if (state == State.IDLE || state == State.DONE) return;

        switch (state) {
            case FIND_AND_FIRE:
                handleFindAndFire();
                break;
            case WAIT_FLIPPER:
                handleWaitFlipper();
                break;
            case ROTATE_SPINNER:
                handleRotateSpinner();
                break;
            case WAIT_SPINNER:
                handleWaitSpinner();
                break;
            case RESET_SPINNER:
                handleResetSpinner();
                break;
            case WAIT_RESET_SPINNER:
                handleWaitResetSpinner();
                break;
        }
    }

    public boolean isBusy() {
        return state != State.IDLE && state != State.DONE;
    }

    public boolean isDone() {
        return state == State.DONE;
    }

    public void setLauncherReadyVelocity(double velocity) {
        this.launcherMinVelocity = velocity;
    }

    //========================================
    // STATE HANDLERS
    //========================================

    private void handleFindAndFire() {
        // Check if all 3 shots done
        if (currentShot >= 3) {
            if (spinnerWasRotated) {
                state = State.RESET_SPINNER;
            } else {
                state = State.DONE;
            }
            return;
        }

        // ===== PURGE MODE =====
        if (purgeMode) {
            if (currentShot == 0) {
                // Shot 0+1: Fire both flippers
                if (waitForLauncher()) {
                    robot.triggerBothFlippers();
                    currentShot = 2; // Skip to shot 2
                    state = State.WAIT_FLIPPER;
                    timer.reset();
                }
            } else if (currentShot == 2) {
                // Shot 2: Need to rotate first, then fire left
                if (!spinnerWasRotated) {
                    state = State.ROTATE_SPINNER;
                } else {
                    // Already rotated, now fire left
                    if (waitForLauncher()) {
                        robot.triggerLeftFlipper();
                        currentShot++;
                        state = State.WAIT_FLIPPER;
                        timer.reset();
                    }
                }
            }
            return;
        }

        // ===== COLOR MATCHING MODE =====
        IntakeSensorFusion002.ArtifactColor targetColor = shotSequence.get(currentShot);
        IntakeSensorFusion002.ArtifactColor left = getLeftColor();
        IntakeSensorFusion002.ArtifactColor right = getRightColor();
        IntakeSensorFusion002.ArtifactColor center = getCenterColor();

        // CASE 1: Target is on left or right (direct fire)
        if (left == targetColor || right == targetColor) {
            if (waitForLauncher()) {
                fireSide(left == targetColor);
                currentShot++;
                state = State.WAIT_FLIPPER;
                timer.reset();
            }
            return;
        }

        // CASE 2: Target is in center (need to rotate)
        if (center == targetColor && !spinnerWasRotated) {
            state = State.ROTATE_SPINNER;
            return;
        }

        // CASE 3: After rotation, ball should be on left
        // (because center rotates to left position)
        if (spinnerWasRotated) {
            // After rotation, just fire whatever is on left
            // (trust that the rotation put the right ball there)

            //TODO: Test this code on tne robot. Removing the following if statement essentially is saying
            // we don't care what the sensor is reading, we assume there's a ball in the left and so
            // we're just going to shoot it. We could also just trigger both flippers, but there
            // won't be a ball in the right, so we really don't need to fire the right flipper.
            // Maybe this will fix the issue we were seeing, where the third ball wasn't firing
            //if (left != IntakeSensorFusion002.ArtifactColor.EMPTY) {
                if (waitForLauncher()) {
                    robot.triggerLeftFlipper();
                    currentShot++;
                    state = State.WAIT_FLIPPER;
                    timer.reset();
                }
                return;
            //}
        }

        // CASE 4: No exact match - fire any available ball as fallback
        // Priority: left > center (rotate) > right
// CASE 4: FALLBACK - No exact match found
        // This is CRITICAL to prevent getting stuck!
        // Strategy: If we haven't rotated yet and center has something, try rotating
        //           Otherwise fire any available ball (prefer left > right)

        robot.telemetry.addData("⚠ FALLBACK", "No exact match for %s", targetColor);

        // Try rotating to center if we haven't yet and center isn't empty
        if (center != IntakeSensorFusion002.ArtifactColor.EMPTY && !spinnerWasRotated) {
            robot.telemetry.addData("  Action", "Rotating to center ball");
            state = State.ROTATE_SPINNER;
            return;
        }

        // Fire any available ball - prefer left over right
        if (left != IntakeSensorFusion002.ArtifactColor.EMPTY) {
            robot.telemetry.addData("  Action", "Firing left (%s)", left);
            if (waitForLauncher()) {
                robot.triggerLeftFlipper();
                currentShot++;
                state = State.WAIT_FLIPPER;
                timer.reset();
            }
        } else if (right != IntakeSensorFusion002.ArtifactColor.EMPTY) {
            robot.telemetry.addData("  Action", "Firing right (%s)", right);
            if (waitForLauncher()) {
                robot.triggerRightFlipper();
                currentShot++;
                state = State.WAIT_FLIPPER;
                timer.reset();
            }
        } else {
            // LAST RESORT: No balls detected anywhere - fire blind and move on
            robot.telemetry.addData("  Action", "NO BALLS DETECTED - Firing blind!");
            if (waitForLauncher()) {
                robot.triggerBothFlippers(); // Fire both by default
                currentShot++;
                state = State.WAIT_FLIPPER;
                timer.reset();
            }
        }
    }

    private void handleWaitFlipper() {
        if (!robot.areFlippersBusy() || timer.seconds() > FLIPPER_TIMEOUT) {
            // After flipper completes, check if we need to reset spinner rotation flag
            // for the NEXT shot (but only in color mode, not at end of sequence)
            if (!purgeMode && currentShot < 3 && spinnerWasRotated) {
                // We used the rotated position for this shot
                // Reset flag so we can rotate again if needed for next shot
                spinnerWasRotated = false;
            }

            state = State.FIND_AND_FIRE;
            timer.reset();
        }
    }

    private void handleRotateSpinner() {
        robot.rotateSpinnerLeft();
        spinnerWasRotated = true;
        state = State.WAIT_SPINNER;
        timer.reset();
    }

    private void handleWaitSpinner() {
        if (!robot.isSpinnerBusy() || timer.seconds() > SPINNER_TIMEOUT) {
            state = State.FIND_AND_FIRE;
            timer.reset();
        }
    }

    private void handleResetSpinner() {
        robot.rotateSpinnerRight();
        state = State.WAIT_RESET_SPINNER;  // Use dedicated wait state
        timer.reset();
    }

    private void handleWaitResetSpinner() {
        if (!robot.isSpinnerBusy() || timer.seconds() > SPINNER_TIMEOUT) {
            state = State.DONE;  // Go directly to DONE
            timer.reset();
        }
    }

    //========================================
    // HELPERS
    //========================================

    /**
     * Wait for launcher to be ready. Returns true if ready or timeout.
     */
    private boolean waitForLauncher() {
        double leftVel = robot.launcher.getLeftMotorVelocity();
        double rightVel = robot.launcher.getRightMotorVelocity();
        double minVel = Math.min(leftVel, rightVel);

        if (minVel >= launcherMinVelocity) {
            return true;
        }

        // Timeout after 2 seconds - fire anyway
        if (timer.seconds() > LAUNCHER_TIMEOUT) {
            robot.telemetry.addData("⚠ Launcher", "Not ready, firing anyway");
            return true;
        }

        return false;
    }

    /**
     * Fire left or right flipper
     */
    private void fireSide(boolean left) {
        if (left) {
            robot.triggerLeftFlipper();
            // Force the sensor system to recognize this slot is now empty
            robot.clearIntakeSlot(IntakeSensorFusion002.IntakeSlot.LEFT);
        } else {
            robot.triggerRightFlipper();
            robot.clearIntakeSlot(IntakeSensorFusion002.IntakeSlot.RIGHT);
        }
    }

    /**
     * Get colors with occupancy check to prevent ghost readings
     */
    private IntakeSensorFusion002.ArtifactColor getLeftColor() {
        if (!robot.isIntakeSlotOccupied(IntakeSensorFusion002.IntakeSlot.LEFT)) {
            return IntakeSensorFusion002.ArtifactColor.EMPTY;
        }
        return robot.getIntakeSlotColor(IntakeSensorFusion002.IntakeSlot.LEFT);
    }

    private IntakeSensorFusion002.ArtifactColor getCenterColor() {
        if (!robot.isIntakeSlotOccupied(IntakeSensorFusion002.IntakeSlot.CENTER)) {
            return IntakeSensorFusion002.ArtifactColor.EMPTY;
        }
        return robot.getIntakeSlotColor(IntakeSensorFusion002.IntakeSlot.CENTER);
    }

    private IntakeSensorFusion002.ArtifactColor getRightColor() {
        if (!robot.isIntakeSlotOccupied(IntakeSensorFusion002.IntakeSlot.RIGHT)) {
            return IntakeSensorFusion002.ArtifactColor.EMPTY;
        }
        return robot.getIntakeSlotColor(IntakeSensorFusion002.IntakeSlot.RIGHT);
    }

    /**
     * Parse motif string
     */
    private List<IntakeSensorFusion002.ArtifactColor> parseMotif(String motif) {
        List<IntakeSensorFusion002.ArtifactColor> seq = new ArrayList<>();
        String m = motif.toUpperCase();

        if (m.length() != 3) m = "PPG"; // Default

        for (int i = 0; i < 3; i++) {
            char c = m.charAt(i);
            seq.add(c == 'G' ?
                    IntakeSensorFusion002.ArtifactColor.GREEN :
                    IntakeSensorFusion002.ArtifactColor.PURPLE);
        }

        return seq;
    }

    //========================================
    // TELEMETRY
    //========================================

    public void addTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        telemetry.addLine("--- SHOT SEQUENCE ---");
        telemetry.addData("Mode", purgeMode ? "PURGE" : "Color Match");
        telemetry.addData("State", state);
        telemetry.addData("Shot", String.format("%d/3", Math.min(currentShot + 1, 3)));

        if (!purgeMode && currentShot < shotSequence.size()) {
            telemetry.addData("Target", shotSequence.get(currentShot));
        }

        telemetry.addData("Colors [L|C|R]", "%s | %s | %s",
                getLeftColor(), getCenterColor(), getRightColor());
        telemetry.addData("Spinner Rotated", spinnerWasRotated);

        double leftVel = robot.launcher.getLeftMotorVelocity();
        double rightVel = robot.launcher.getRightMotorVelocity();
        boolean leftReady = leftVel >= launcherMinVelocity;
        boolean rightReady = rightVel >= launcherMinVelocity;

        telemetry.addData("Launcher L/R", "%s %.0f | %s %.0f",
                leftReady ? "✓" : "⚠", leftVel,
                rightReady ? "✓" : "⚠", rightVel);
    }
}