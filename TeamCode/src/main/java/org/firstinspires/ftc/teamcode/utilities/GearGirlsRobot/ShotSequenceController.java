package org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

/**
 * ShotSequenceController - Sensor-Based 3-Shot Sequence Manager
 * ==============================================================
 *
 * This controller manages the complete sequence of firing 3 shots based on a motif pattern (GPP, PGP, or PPG).
 * It handles:
 * - Reading color sensors to find the correct ball
 * - Rotating the spinner if needed to position the ball
 * - Firing the correct flipper (left or right)
 * - Waiting for each shot to complete before moving to the next
 *
 * The controller uses a simple state machine to coordinate these actions and ensures
 * the launcher is at speed before firing.
 *
 * OPTIMIZATIONS (v3.2):
 * ---------------------
 * - Sensor readings cached once per update() cycle (eliminates redundant I2C reads)
 * - Motor velocity checked for correct side (left vs right) before firing
 * - Dual-launcher optimization: only waits for the motor that will actually fire
 *
 * PURGE MODE:
 * -----------
 * Use purge() to fire ALL remaining balls quickly without color matching.
 * Perfect for:
 * - End of autonomous (clear inventory before parking)
 * - Before collecting more balls (avoid penalty for >3 balls)
 * - Backup when sensors fail completely
 *
 * IMPORTANT - Multiple Sequences:
 * -------------------------------
 * The controller can be reused multiple times in autonomous. Simply call start() again
 * after isDone() returns true. Each call to start() resets the controller completely.
 *
 * Example Usage in Autonomous (Multiple Volleys):
 * -----------------------------------------------
 * // First volley (preload):
 * case SHOOT_SEQUENCE:
 *     if (!robot.shotSequence.isBusy()) {
 *         robot.shotSequence.start("GPP");
 *     }
 *     if (robot.shotSequence.isDone()) {
 *         state = COLLECT_SPIKE_MARK;  // Move to next state
 *     }
 *     break;
 *
 * // Before collecting - check inventory
 * case BEFORE_COLLECT:
 *     if (robot.getIntakeInventoryCount() >= 2) {
 *         // Need to purge first to avoid penalty
 *         robot.shotSequence.purge();
 *         state = WAIT_PURGE;
 *     } else {
 *         state = COLLECT;
 *     }
 *     break;
 *
 * case WAIT_PURGE:
 *     if (robot.shotSequence.isDone()) {
 *         robot.shotSequence.reset();
 *         state = COLLECT;
 *     }
 *     break;
 *
 * // End of auto - purge everything
 * case END_AUTO:
 *     robot.shotSequence.purge();
 *     state = WAIT_FINAL_PURGE;
 *     break;
 *
 * @author GearGirls Team
 * @version 3.2 - Optimized sensor caching and dual-motor velocity checks
 */
public class ShotSequenceController {

    // --- State Machine ---
    private enum State {
        IDLE,                    // Not running
        POSITION_BALL,          // Finding the correct color
        WAIT_SPINNER,           // Waiting for spinner to finish rotating
        FIRE_BOTH,              // Firing both flippers simultaneously (blind mode OR dual-purple optimization)
        WAIT_BOTH_FLIPPERS,     // Waiting for both flippers to finish
        FIRE_SHOT,              // Firing a single flipper
        WAIT_FLIPPER,           // Waiting for flipper to finish
        RESET_SPINNER,          // Rotating spinner back to home position
        WAIT_RESET_SPINNER,     // Waiting for reset rotation to complete
        DONE,                   // All 3 shots complete
        ERROR                   // Something went wrong
    }

    // --- Robot Reference ---
    private final GGRobot2 robot;

    // --- State Variables ---
    private State currentState = State.IDLE;
    private List<IntakeSensorFusion002.ArtifactColor> shotSequence = new ArrayList<>();
    private int currentShot = 0; // 0, 1, or 2
    private int spinnerAttempts = 0;
    private String motifPattern = "";
    private String errorMessage = "";
    private boolean spinnerWasRotated = false; // Track if spinner was rotated during this sequence
    private boolean purgeMode = false;  // Track if we're in purge mode

    // --- CACHED SENSOR READINGS (NEW v3.2) ---
    // Read once per update() cycle to eliminate redundant I2C transactions
    private IntakeSensorFusion002.ArtifactColor cachedLeftColor;
    private IntakeSensorFusion002.ArtifactColor cachedCenterColor;
    private IntakeSensorFusion002.ArtifactColor cachedRightColor;

    // --- Timers ---
    private final ElapsedTime stateTimer = new ElapsedTime();

    // --- Settings (can be adjusted) ---
    private double launcherMinVelocity = 1400.0;     // Minimum velocity (ticks/sec) before firing
    private boolean requireLauncherReady = true;     // Wait for launcher before firing
    private boolean blindFireMode = false;           // If true, fire L+R simultaneously then spin+L
    private static final double SPINNER_TIMEOUT_SEC = 0.8;
    private static final double FLIPPER_TIMEOUT_SEC = 1.50;
    private static final int MAX_SPINNER_ATTEMPTS = 1;

    /**
     * Constructor - called once when GGRobot2 is created
     */
    public ShotSequenceController(GGRobot2 robot) {
        this.robot = robot;
    }

    //================================================================================
    // PUBLIC API - Called by Autonomous
    //================================================================================

    /**
     * Start a new 3-shot sequence.
     * This automatically resets the controller, so it can be called multiple times
     * throughout autonomous (e.g., after collecting more balls).
     *
     * @param motif The shot pattern: "GPP", "PGP", or "PPG"
     */
    public void start(String motif) {
        // Reset all state variables for a fresh start
        this.motifPattern = motif == null ? "PPG" : motif.toUpperCase();
        this.shotSequence = parseMotif(this.motifPattern);
        this.currentShot = 0;
        this.spinnerAttempts = 0;
        this.errorMessage = "";
        this.spinnerWasRotated = false;
        this.purgeMode = false;
        this.currentState = State.POSITION_BALL;
        stateTimer.reset();
    }

    /**
     * Purge all remaining balls quickly without color matching.
     *
     * This uses the fastest possible firing pattern:
     * 1. Fire both flippers simultaneously (clears 2 balls)
     * 2. Rotate spinner left
     * 3. Fire left flipper (clears last ball)
     * 4. Reset spinner to home
     *
     * Use cases:
     * - End of autonomous to clear inventory before parking
     * - Before collecting more balls to avoid >3 ball penalty
     * - Backup when sensors fail and you just need to shoot everything
     */
    public void purge() {
        this.motifPattern = "PURGE";
        this.shotSequence.clear();  // Not using motif pattern in purge mode
        this.currentShot = 0;
        this.spinnerAttempts = 0;
        this.errorMessage = "";
        this.spinnerWasRotated = false;
        this.purgeMode = true;
        this.currentState = State.FIRE_BOTH;  // Start by firing both
        stateTimer.reset();
    }

    /**
     * Stop the current sequence and return to IDLE.
     */
    public void cancel() {
        currentState = State.IDLE;
        currentShot = 0;
        spinnerAttempts = 0;
        errorMessage = "Cancelled by user";
        spinnerWasRotated = false;
        purgeMode = false;
    }

    /**
     * Reset the controller to IDLE state, clearing all errors and completion flags.
     * This should be called between volleys in autonomous to ensure a clean start.
     *
     * Example usage:
     * if (robot.shotSequence.isDone()) {
     *     robot.shotSequence.reset();  // Clear done flag
     *     state = NEXT_STATE;
     * }
     */
    public void reset() {
        currentState = State.IDLE;
        currentShot = 0;
        spinnerAttempts = 0;
        errorMessage = "";
        motifPattern = "";
        shotSequence.clear();
        spinnerWasRotated = false;
        purgeMode = false;
    }

    /**
     * Call this every loop - it advances the state machine.
     * Safe to call always; does nothing when IDLE.
     *
     * OPTIMIZATION (v3.2): Sensor readings are cached once per update cycle
     * to eliminate redundant I2C transactions.
     */
    public void update() {
        if (currentState == State.IDLE || currentState == State.DONE) {
            return;
        }

        // Safety check (skip for purge mode since it doesn't use shot sequence)
        if (!purgeMode && shotSequence.size() != 3) {
            currentState = State.ERROR;
            errorMessage = "Invalid motif pattern";
            return;
        }

        // *** CACHE SENSOR READINGS ONCE PER UPDATE CYCLE (v3.2) ***
        cachedLeftColor = robot.getIntakeSlotColor(IntakeSensorFusion002.IntakeSlot.LEFT);
        cachedCenterColor = robot.getIntakeSlotColor(IntakeSensorFusion002.IntakeSlot.CENTER);
        cachedRightColor = robot.getIntakeSlotColor(IntakeSensorFusion002.IntakeSlot.RIGHT);

        // *** GHOST READING PROTECTION (NEW v3.3) ***
        // If occupancy sensor says EMPTY, override color reading with EMPTY
        // This prevents firing on ghost readings from recently-fired balls
        if (!robot.isIntakeSlotOccupied(IntakeSensorFusion002.IntakeSlot.LEFT)) {
            cachedLeftColor = IntakeSensorFusion002.ArtifactColor.EMPTY;
        }
        if (!robot.isIntakeSlotOccupied(IntakeSensorFusion002.IntakeSlot.CENTER)) {
            cachedCenterColor = IntakeSensorFusion002.ArtifactColor.EMPTY;
        }
        if (!robot.isIntakeSlotOccupied(IntakeSensorFusion002.IntakeSlot.RIGHT)) {
            cachedRightColor = IntakeSensorFusion002.ArtifactColor.EMPTY;
        }

        // Run the state machine
        switch (currentState) {
            case POSITION_BALL:
                handlePositionBall();
                break;

            case WAIT_SPINNER:
                handleWaitSpinner();
                break;

            case FIRE_BOTH:
                handleFireBoth();
                break;

            case WAIT_BOTH_FLIPPERS:
                handleWaitBothFlippers();
                break;

            case FIRE_SHOT:
                handleFireShot();
                break;

            case WAIT_FLIPPER:
                handleWaitFlipper();
                break;

            case RESET_SPINNER:
                handleResetSpinner();
                break;

            case WAIT_RESET_SPINNER:
                handleWaitResetSpinner();
                break;

            default:
                break;
        }
    }

    /**
     * Returns true while a sequence is in progress.
     */
    public boolean isBusy() {
        return currentState != State.IDLE &&
                currentState != State.DONE &&
                currentState != State.ERROR;
    }

    /**
     * Returns true when all 3 shots are complete.
     */
    public boolean isDone() {
        return currentState == State.DONE;
    }

    /**
     * Returns true if an error occurred.
     */
    public boolean isError() {
        return currentState == State.ERROR;
    }

    /**
     * Returns true if currently in purge mode.
     */
    public boolean isPurging() {
        return purgeMode;
    }

    //================================================================================
    // SETTINGS
    //================================================================================

    /**
     * Set the minimum launcher velocity (ticks/sec) required before firing.
     */
    public void setLauncherReadyVelocity(double velocity) {
        this.launcherMinVelocity = velocity;
    }

    /**
     * Enable/disable waiting for launcher to be at speed.
     */
    public void setRequireLauncherReady(boolean require) {
        this.requireLauncherReady = require;
    }

    /**
     * Enable/disable blind fire mode.
     *
     * When enabled, the controller ignores colors and fires using the fastest possible pattern,
     * taking advantage of the two separate shooters:
     *   Shots 1+2: Fire BOTH flippers simultaneously (one flipper wait cycle)
     *   Rotate spinner left to access middle ball
     *   Shot 3:    Fire LEFT flipper
     *   Reset spinner back to home
     *
     * This is useful as a backup strategy if sensors aren't working reliably,
     * and is also faster than sensor mode since shots 1+2 are combined.
     *
     * @param enabled true to enable blind fire mode, false for sensor-based mode
     */
    public void setBlindFireMode(boolean enabled) {
        this.blindFireMode = enabled;
    }



    //================================================================================
    // STATE MACHINE HANDLERS
    //================================================================================

    private void handlePositionBall() {
        // Check if we've completed all 3 shots
        if (currentShot >= 3) {
            currentState = State.DONE;
            return;
        }

        // BLIND FIRE MODE: L+R simultaneous, then spin, then L
        if (blindFireMode) {
            if (currentShot == 0) {
                currentState = State.FIRE_BOTH;
                stateTimer.reset();
            } else if (currentShot == 2 && spinnerAttempts == 0) {
                robot.rotateSpinnerLeft();
                spinnerWasRotated = true;
                spinnerAttempts++;
                currentState = State.WAIT_SPINNER;
                stateTimer.reset();
            } else {
                currentState = State.FIRE_SHOT;
                stateTimer.reset();
            }
            return;
        }

        // === SENSOR-BASED MODE ===
        // Use cached sensor values (read once per update cycle)
        IntakeSensorFusion002.ArtifactColor desiredColor = shotSequence.get(currentShot);
        IntakeSensorFusion002.ArtifactColor leftColor = cachedLeftColor;
        IntakeSensorFusion002.ArtifactColor centerColor = cachedCenterColor;
        IntakeSensorFusion002.ArtifactColor rightColor = cachedRightColor;

        // === DUAL-PURPLE OPTIMIZATION ===
        if (currentShot == 0 && spinnerAttempts == 0) {
            IntakeSensorFusion002.ArtifactColor shot0 = shotSequence.get(0);
            IntakeSensorFusion002.ArtifactColor shot1 = shotSequence.get(1);

            if (shot0 == IntakeSensorFusion002.ArtifactColor.PURPLE &&
                    shot1 == IntakeSensorFusion002.ArtifactColor.PURPLE &&
                    leftColor == IntakeSensorFusion002.ArtifactColor.PURPLE &&
                    rightColor == IntakeSensorFusion002.ArtifactColor.PURPLE) {

                currentState = State.FIRE_BOTH;
                stateTimer.reset();
                return;
            }
        }
        // === END DUAL-PURPLE OPTIMIZATION ===

        // === CHECK ALL THREE SENSORS DIRECTLY ===
        boolean foundOnLeft = (leftColor == desiredColor);
        boolean foundOnCenter = (centerColor == desiredColor);
        boolean foundOnRight = (rightColor == desiredColor);

        if (foundOnLeft || foundOnRight) {
            spinnerAttempts = 0;
            currentState = State.FIRE_SHOT;
            stateTimer.reset();
            return;
        }

        if (foundOnCenter) {
            if (spinnerAttempts == 0) {
                robot.rotateSpinnerLeft();
                spinnerWasRotated = true;
                spinnerAttempts++;
                currentState = State.WAIT_SPINNER;
                stateTimer.reset();
            } else {
                currentState = State.FIRE_SHOT;
                stateTimer.reset();
            }
            return;
        }
        // === END DIRECT SENSOR CHECK ===

        // === DEDUCTION LOGIC FOR UNKNOWN CASES ===
        if (spinnerAttempts == 0) {
            IntakeSensorFusion002.ArtifactColor deducedColor = deduceThirdBallWithCenter(leftColor, centerColor, rightColor);

            if (deducedColor == desiredColor && deducedColor != IntakeSensorFusion002.ArtifactColor.UNKNOWN) {

                if (leftColor == IntakeSensorFusion002.ArtifactColor.UNKNOWN) {
                    currentState = State.FIRE_SHOT;
                    stateTimer.reset();
                    return;
                }

                if (centerColor == IntakeSensorFusion002.ArtifactColor.UNKNOWN) {
                    robot.rotateSpinnerLeft();
                    spinnerWasRotated = true;
                    spinnerAttempts++;
                    currentState = State.WAIT_SPINNER;
                    stateTimer.reset();
                    return;
                }

                if (rightColor == IntakeSensorFusion002.ArtifactColor.UNKNOWN) {
                    currentState = State.FIRE_SHOT;
                    stateTimer.reset();
                    return;
                }
            }
        }
        // === END DEDUCTION LOGIC ===

        // === FALLBACK: BLIND ROTATION ===
        if (spinnerAttempts >= MAX_SPINNER_ATTEMPTS) {
            errorMessage = String.format("Cannot find %s, firing anyway", desiredColor);
            currentState = State.FIRE_SHOT;
            stateTimer.reset();
            return;
        }

        if (spinnerAttempts % 2 == 0) {
            robot.rotateSpinnerLeft();
            spinnerWasRotated = true;
        } else {
            robot.rotateSpinnerRight();
        }

        spinnerAttempts++;
        currentState = State.WAIT_SPINNER;
        stateTimer.reset();
    }

    /**
     * WAIT_SPINNER: Wait for spinner rotation to complete.
     */
    private void handleWaitSpinner() {
        if (!robot.isSpinnerBusy()) {
            // Spinner finished - go back to positioning (or FIRE_SHOT in purge mode)
            if (purgeMode && currentShot == 2) {
                // In purge mode after spinning for shot 2, go straight to firing
                currentState = State.FIRE_SHOT;
            } else {
                currentState = State.POSITION_BALL;
            }
            stateTimer.reset();
        } else if (stateTimer.seconds() > SPINNER_TIMEOUT_SEC) {
            // Timeout
            if (purgeMode && currentShot == 2) {
                // In purge mode, fire anyway
                currentState = State.FIRE_SHOT;
            } else {
                currentState = State.ERROR;
                errorMessage = "Spinner timeout";
            }
        }
    }

    /**
     * FIRE_BOTH: Fire both flippers simultaneously.
     * Used for:
     * - Blind fire mode (shots 0+1)
     * - Dual-purple optimization in sensor mode (when P/P visible and next 2 shots are P/P)
     * - Purge mode (first action)
     *
     * OPTIMIZATION (v3.2): Waits for BOTH launcher motors to be at speed since both will fire.
     */
    private void handleFireBoth() {
        // Wait for BOTH launchers to be at speed (since we're firing both flippers)
        if (requireLauncherReady) {
            double leftVelocity = robot.launcher.getLeftMotorVelocity();
            double rightVelocity = robot.launcher.getRightMotorVelocity();

            if (leftVelocity < launcherMinVelocity || rightVelocity < launcherMinVelocity) {
                if (stateTimer.seconds() > 2.0) {
                    errorMessage = "Launcher not ready, firing anyway";
                } else {
                    return; // Keep waiting for BOTH motors
                }
            }
        }

        robot.triggerBothFlippers();
        currentState = State.WAIT_BOTH_FLIPPERS;
        stateTimer.reset();
    }

    /**
     * WAIT_BOTH_FLIPPERS: Wait for both flippers to finish, then advance currentShot past both.
     * After this we go to shot 2 logic (spinner rotate + single fire) in both normal and purge modes.
     */
    private void handleWaitBothFlippers() {
        if (!robot.areFlippersBusy()) {
            currentShot = 2;  // Shots 0 and 1 are done
            spinnerAttempts = 0;

            // In purge mode, rotate immediately for shot 2
            if (purgeMode) {
                robot.rotateSpinnerLeft();
                spinnerWasRotated = true;
                spinnerAttempts++;
                currentState = State.WAIT_SPINNER;
            } else {
                currentState = State.POSITION_BALL;
            }
            stateTimer.reset();
        } else if (stateTimer.seconds() > FLIPPER_TIMEOUT_SEC) {
            currentShot = 2;  // Advance anyway
            spinnerAttempts = 0;

            // In purge mode, rotate immediately for shot 2
            if (purgeMode) {
                robot.rotateSpinnerLeft();
                spinnerWasRotated = true;
                spinnerAttempts++;
                currentState = State.WAIT_SPINNER;
            } else {
                currentState = State.POSITION_BALL;
            }
            stateTimer.reset();
            errorMessage = "Both flippers timeout";
        }
    }

    /**
     * FIRE_SHOT: Fire the correct flipper once launcher is ready.
     *
     * OPTIMIZATION (v3.2): Determines which flipper will fire FIRST, then checks
     * the velocity of only that motor. This prevents wasting time waiting for
     * the wrong motor to spin up.
     */
    private void handleFireShot() {
        // Determine which flipper we're going to fire FIRST
        boolean fireLeft = false;
        boolean fireRight = false;

        if (blindFireMode || purgeMode) {
            // Shot 2 after spinner rotation - always LEFT
            fireLeft = true;
        } else {
            // SENSOR-BASED MODE: Determine which side to fire using cached sensor values
            IntakeSensorFusion002.ArtifactColor desiredColor = shotSequence.get(currentShot);
            IntakeSensorFusion002.ArtifactColor leftColor = cachedLeftColor;
            IntakeSensorFusion002.ArtifactColor rightColor = cachedRightColor;

            // Priority order for which flipper to fire:
            // 1. Side that matches desired color
            // 2. Side that is occupied (not EMPTY, even if UNKNOWN)
            // 3. Error if no balls present

            if (leftColor == desiredColor) {
                fireLeft = true;
            } else if (rightColor == desiredColor) {
                fireRight = true;
            } else if (leftColor != IntakeSensorFusion002.ArtifactColor.EMPTY &&
                    leftColor != IntakeSensorFusion002.ArtifactColor.UNKNOWN) {
                // Left side has a colored ball (PURPLE or GREEN) - fire it
                fireLeft = true;
            } else if (rightColor != IntakeSensorFusion002.ArtifactColor.EMPTY &&
                    rightColor != IntakeSensorFusion002.ArtifactColor.UNKNOWN) {
                // Right side has a colored ball (PURPLE or GREEN) - fire it
                fireRight = true;
            } else if (leftColor == IntakeSensorFusion002.ArtifactColor.UNKNOWN &&
                    robot.isIntakeSlotOccupied(IntakeSensorFusion002.IntakeSlot.LEFT)) {
                // Left side UNKNOWN but physically occupied - fire it
                fireLeft = true;
            } else if (rightColor == IntakeSensorFusion002.ArtifactColor.UNKNOWN &&
                    robot.isIntakeSlotOccupied(IntakeSensorFusion002.IntakeSlot.RIGHT)) {
                // Right side UNKNOWN but physically occupied - fire it
                fireRight = true;
            } else {
                // *** SAFETY: No balls detected - ERROR instead of firing empty ***
                errorMessage = String.format("No balls to fire! L=%s R=%s", leftColor, rightColor);
                currentState = State.ERROR;
                return;
            }
        }

        // NOW check the velocity of the CORRECT motor (v3.2)
        if (requireLauncherReady) {
            double motorVelocity;

            if (fireLeft) {
                motorVelocity = robot.launcher.getLeftMotorVelocity();
            } else {
                motorVelocity = robot.launcher.getRightMotorVelocity();
            }

            if (motorVelocity < launcherMinVelocity) {
                if (stateTimer.seconds() > 2.0) {
                    errorMessage = "Launcher not ready, firing anyway";
                } else {
                    return; // Keep waiting for the CORRECT motor
                }
            }
        }

        // Fire the determined flipper
        if (fireLeft) {
            robot.triggerLeftFlipper();
            cachedLeftColor = IntakeSensorFusion002.ArtifactColor.EMPTY;
        } else {
            robot.triggerRightFlipper();
            cachedRightColor = IntakeSensorFusion002.ArtifactColor.EMPTY;
        }

        currentState = State.WAIT_FLIPPER;
        stateTimer.reset();
    }

    /**
     * WAIT_FLIPPER: Wait for flipper cycle to complete.
     */
    private void handleWaitFlipper() {
        if (!robot.areFlippersBusy()) {
            // Flipper finished - advance to next shot
            currentShot++;

            // Check if all shots are done
            if (currentShot >= 3) {
                // All 3 shots complete - reset spinner if it was rotated
                if (spinnerWasRotated) {
                    currentState = State.RESET_SPINNER;
                } else {
                    currentState = State.DONE;
                }
            } else {
                // More shots remaining
                spinnerAttempts = 0;
                currentState = State.POSITION_BALL;
            }
            stateTimer.reset();
        } else if (stateTimer.seconds() > FLIPPER_TIMEOUT_SEC) {
            // Timeout - advance anyway to keep auto moving
            currentShot++;

            // Check if all shots are done
            if (currentShot >= 3) {
                // All 3 shots complete - reset spinner if it was rotated
                if (spinnerWasRotated) {
                    currentState = State.RESET_SPINNER;
                } else {
                    currentState = State.DONE;
                }
            } else {
                // More shots remaining
                spinnerAttempts = 0;
                currentState = State.POSITION_BALL;
            }
            stateTimer.reset();
            errorMessage = "Flipper timeout";
        }
    }

    /**
     * RESET_SPINNER: Rotate spinner back to home position (right) after completing shots.
     * This ensures the spinner is ready for the next volley.
     */
    private void handleResetSpinner() {
        robot.rotateSpinnerRight(); // Rotate back to home position
        currentState = State.WAIT_RESET_SPINNER;
        stateTimer.reset();
    }

    /**
     * WAIT_RESET_SPINNER: Wait for reset rotation to complete.
     */
    private void handleWaitResetSpinner() {
        if (!robot.isSpinnerBusy()) {
            // Reset rotation finished - now we're done
            currentState = State.DONE;
            stateTimer.reset();
        } else if (stateTimer.seconds() > SPINNER_TIMEOUT_SEC) {
            // Timeout - just mark as done anyway
            currentState = State.DONE;
            stateTimer.reset();
        }
    }

    //================================================================================
    // HELPER METHODS
    //================================================================================

    /**
     * Convert motif string (e.g., "GPP") into a list of colors.
     */
    private List<IntakeSensorFusion002.ArtifactColor> parseMotif(String motif) {
        List<IntakeSensorFusion002.ArtifactColor> sequence = new ArrayList<>(3);

        if (motif == null || motif.length() != 3) {
            // Default to PPG
            sequence.add(IntakeSensorFusion002.ArtifactColor.PURPLE);
            sequence.add(IntakeSensorFusion002.ArtifactColor.PURPLE);
            sequence.add(IntakeSensorFusion002.ArtifactColor.GREEN);
            return sequence;
        }

        for (int i = 0; i < 3; i++) {
            char c = motif.charAt(i);
            if (c == 'G') {
                sequence.add(IntakeSensorFusion002.ArtifactColor.GREEN);
            } else if (c == 'P') {
                sequence.add(IntakeSensorFusion002.ArtifactColor.PURPLE);
            } else {
                // Invalid character - return default
                sequence.clear();
                sequence.add(IntakeSensorFusion002.ArtifactColor.PURPLE);
                sequence.add(IntakeSensorFusion002.ArtifactColor.PURPLE);
                sequence.add(IntakeSensorFusion002.ArtifactColor.GREEN);
                return sequence;
            }
        }

        return sequence;
    }

    /**
     * Get friendly state name for telemetry.
     */
    private String getStateName() {
        switch (currentState) {
            case IDLE: return "Idle";
            case POSITION_BALL: return purgeMode ? "Purging..." : "Positioning Ball";
            case WAIT_SPINNER: return "Waiting for Spinner";
            case FIRE_BOTH: return purgeMode ? "Purge: Firing L+R" : "Firing Both";
            case WAIT_BOTH_FLIPPERS: return "Waiting for Both";
            case FIRE_SHOT: return purgeMode ? "Purge: Firing Last" : "Firing Shot";
            case WAIT_FLIPPER: return "Waiting for Flipper";
            case RESET_SPINNER: return "Resetting Spinner";
            case WAIT_RESET_SPINNER: return "Waiting for Reset";
            case DONE: return "✓ COMPLETE ✓";
            case ERROR: return "⚠ ERROR";
            default: return "Unknown";
        }
    }

    /**
     * Helper: Deduce the third ball color given left and right sensor readings (original version).
     * Assumes: exactly 2 PURPLE + 1 GREEN total.
     * Used as fallback when center sensor is not available.
     *
     * @return The deduced color, or UNKNOWN if sensors are unclear
     */
    public static IntakeSensorFusion002.ArtifactColor deduceThirdBall(
            IntakeSensorFusion002.ArtifactColor left,
            IntakeSensorFusion002.ArtifactColor right) {

        // If either sensor can't read, we can't deduce
        if (left == IntakeSensorFusion002.ArtifactColor.UNKNOWN ||
                right == IntakeSensorFusion002.ArtifactColor.UNKNOWN) {
            return IntakeSensorFusion002.ArtifactColor.UNKNOWN;
        }

        // Both purple → third must be green
        if (left == IntakeSensorFusion002.ArtifactColor.PURPLE &&
                right == IntakeSensorFusion002.ArtifactColor.PURPLE) {
            return IntakeSensorFusion002.ArtifactColor.GREEN;
        }

        // One of each → third must be purple (since we need 2 purple total)
        if ((left == IntakeSensorFusion002.ArtifactColor.GREEN && right == IntakeSensorFusion002.ArtifactColor.PURPLE) ||
                (left == IntakeSensorFusion002.ArtifactColor.PURPLE && right == IntakeSensorFusion002.ArtifactColor.GREEN)) {
            return IntakeSensorFusion002.ArtifactColor.PURPLE;
        }

        // Both green → impossible with our ball set
        return IntakeSensorFusion002.ArtifactColor.UNKNOWN;
    }

    /**
     * ENHANCED: Deduce the missing ball color when we can read 2 out of 3 sensors.
     * With the center sensor, we have much better deduction capability!
     *
     * @param left Color reading from left slot (or UNKNOWN)
     * @param center Color reading from center slot (or UNKNOWN)
     * @param right Color reading from right slot (or UNKNOWN)
     * @return The deduced color of the UNKNOWN slot, or UNKNOWN if we can't deduce
     */
    private static IntakeSensorFusion002.ArtifactColor deduceThirdBallWithCenter(
            IntakeSensorFusion002.ArtifactColor left,
            IntakeSensorFusion002.ArtifactColor center,
            IntakeSensorFusion002.ArtifactColor right) {

        // Count how many sensors can actually read a color
        int unknownCount = 0;
        if (left == IntakeSensorFusion002.ArtifactColor.UNKNOWN) unknownCount++;
        if (center == IntakeSensorFusion002.ArtifactColor.UNKNOWN) unknownCount++;
        if (right == IntakeSensorFusion002.ArtifactColor.UNKNOWN) unknownCount++;

        // If more than one sensor is UNKNOWN, we can't deduce reliably
        if (unknownCount > 1) {
            return IntakeSensorFusion002.ArtifactColor.UNKNOWN;
        }

        // If all sensors work, no need to deduce
        if (unknownCount == 0) {
            return IntakeSensorFusion002.ArtifactColor.UNKNOWN;
        }

        // At this point, exactly ONE sensor is UNKNOWN
        // We can deduce what it must be based on the constraint: 2 PURPLE + 1 GREEN

        // Count purples and greens we can see
        int purpleCount = 0;
        int greenCount = 0;

        if (left == IntakeSensorFusion002.ArtifactColor.PURPLE) purpleCount++;
        if (left == IntakeSensorFusion002.ArtifactColor.GREEN) greenCount++;
        if (center == IntakeSensorFusion002.ArtifactColor.PURPLE) purpleCount++;
        if (center == IntakeSensorFusion002.ArtifactColor.GREEN) greenCount++;
        if (right == IntakeSensorFusion002.ArtifactColor.PURPLE) purpleCount++;
        if (right == IntakeSensorFusion002.ArtifactColor.GREEN) greenCount++;

        // Deduce the missing ball
        // We know: total must be 2 purple + 1 green
        if (purpleCount == 2) {
            // Already have 2 purples → third must be green
            return IntakeSensorFusion002.ArtifactColor.GREEN;
        } else if (greenCount == 1) {
            // Already have 1 green → third must be purple
            return IntakeSensorFusion002.ArtifactColor.PURPLE;
        } else if (purpleCount == 1 && greenCount == 1) {
            // Have 1 of each → third must be purple (to reach 2 total)
            return IntakeSensorFusion002.ArtifactColor.PURPLE;
        } else {
            // Shouldn't happen with valid ball configuration
            return IntakeSensorFusion002.ArtifactColor.UNKNOWN;
        }
    }

    //================================================================================
    // TELEMETRY - Familiar GearGirls Style
    //================================================================================

    /**
     * Add shot sequence telemetry to the driver station.
     * Uses the familiar format the team is used to.
     */
    public void addTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        telemetry.addLine("--- SHOT SEQUENCE ---");
        telemetry.addData("Motif Pattern", motifPattern);

        // Show mode clearly
        String mode = purgeMode ? "PURGE" : (blindFireMode ? "BLIND FIRE" : "Sensor-Based");
        telemetry.addData("Mode", mode);

        telemetry.addData("State", getStateName());

        if (!purgeMode) {
            telemetry.addData("Shot Progress", String.format("%d/3", currentShot + 1));
            if (currentShot < shotSequence.size()) {
                telemetry.addData("Next Color", shotSequence.get(currentShot));
            }
        } else {
            telemetry.addData("Purge Progress", currentShot == 0 ? "Firing L+R" :
                    currentShot == 2 ? "Firing Last" : "Rotating");
        }

        // Sensor readings - use cached values for consistency
        telemetry.addData("Sensors [L|C|R]", "%s | %s | %s", cachedLeftColor, cachedCenterColor, cachedRightColor);

        // Status indicators
        if (spinnerAttempts > 0) {
            telemetry.addData("Spinner Attempts", spinnerAttempts);
        }

        boolean spinnerBusy = robot.isSpinnerBusy();
        boolean flippersBusy = robot.areFlippersBusy();
        telemetry.addData("Spinner", spinnerBusy ? "BUSY" : "READY");
        telemetry.addData("Flippers", flippersBusy ? "BUSY" : "READY");

        // Launcher status - show BOTH motors (NEW v3.2)
        double leftVelocity = robot.launcher.getLeftMotorVelocity();
        double rightVelocity = robot.launcher.getRightMotorVelocity();
        boolean leftReady = leftVelocity >= launcherMinVelocity;
        boolean rightReady = rightVelocity >= launcherMinVelocity;

        telemetry.addData("Launcher L/R Velocity", String.format("%.0f / %.0f ticks/sec", leftVelocity, rightVelocity));
        telemetry.addData("Launcher L/R Ready",
                String.format("%s / %s",
                        leftReady ? "✓" : "⚠",
                        rightReady ? "✓" : "⚠"));

        // Error message if present
        if (currentState == State.ERROR && !errorMessage.isEmpty()) {
            telemetry.addData("⚠ ERROR", errorMessage);
        }

        telemetry.addLine();
    }
}