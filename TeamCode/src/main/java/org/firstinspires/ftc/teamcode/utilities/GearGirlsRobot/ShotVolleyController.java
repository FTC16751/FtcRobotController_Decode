package org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * ShotVolleyController
 * -------------------
 * Reusable micro-state-machine for executing a 3-shot volley.
 *
 * Why this exists:
 *  - Keeps Autonomous readable (drive/intake/score calls, not micro timing)
 *  - Reusable in TeleOp (e.g., “press X = shoot motif volley”)
 *  - Single source of truth for: sensor reads, spinner indexing, flipper timing
 *
 * This controller does NOT own hardware directly; it calls into GGRobot2’s high-level
 * primitives (rotateSpinnerLeft/Right, triggerLeft/RightFlipper, getIntakeSlotColor, etc.).
 *
 * NOTE:
 *  - We only have LEFT + RIGHT color sensors.
 *  - We assume the 3-ball set always contains exactly 2 PURPLE and 1 GREEN.
 *  - The “third” ball color can be deduced if needed, but we primarily operate by
 *    making sure the NEXT desired color is visible on LEFT or RIGHT before firing.
 */
public class ShotVolleyController {

    public enum Stage {
        IDLE,
        POSITIONING_FOR_SHOT,
        WAITING_FOR_SPINNER,
        FIRING_FLIPPER,
        WAITING_FOR_FLIPPER,
        DONE,
        ERROR
    }

    public static class Status {
        public Stage stage = Stage.IDLE;
        public String motif = "";
        public int shotIndex = 0; // 0..2
        public IntakeSensorFusion002.ArtifactColor desired = IntakeSensorFusion002.ArtifactColor.UNKNOWN;
        public IntakeSensorFusion002.ArtifactColor left = IntakeSensorFusion002.ArtifactColor.UNKNOWN;
        public IntakeSensorFusion002.ArtifactColor right = IntakeSensorFusion002.ArtifactColor.UNKNOWN;
        public int spinnerAttempts = 0;
        public boolean spinnerBusy = false;
        public boolean flippersBusy = false;
        public double launcherRpm = 0.0;

        public String debugNote = "";
    }

    private final GGRobot2 robot;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final Status status = new Status();

    private List<IntakeSensorFusion002.ArtifactColor> desiredSequence = Collections.emptyList();
    private boolean active = false;

    // Tunables (safe defaults)
    private double launcherReadyRpm = 500.0;
    private double spinnerTimeoutMs = 600.0;
    private double flipperTimeoutMs = 800.0;

    // If true, we will wait for launcher RPM before firing each shot
    private boolean requireLauncherReady = true;

    public ShotVolleyController(GGRobot2 robot) {
        this.robot = robot;
    }

    // ---------------- Public API ----------------

    /** Start a new 3-shot volley for the given motif string (e.g., "GPP"). */
    public void start(String motif) {
        // Reset
        this.desiredSequence = motifToSequence(motif);
        this.active = true;

        status.motif = motif == null ? "" : motif;
        status.shotIndex = 0;
        status.spinnerAttempts = 0;
        status.stage = Stage.POSITIONING_FOR_SHOT;
        status.debugNote = "";

        stateTimer.reset();
    }

    /** Abort and return to IDLE. */
    public void cancel() {
        active = false;
        desiredSequence = Collections.emptyList();
        status.stage = Stage.IDLE;
        status.shotIndex = 0;
        status.spinnerAttempts = 0;
        status.desired = IntakeSensorFusion002.ArtifactColor.UNKNOWN;
        status.debugNote = "cancelled";
    }

    /** True while a volley is in progress (not IDLE/DONE). */
    public boolean isBusy() {
        return active && (status.stage != Stage.DONE) && (status.stage != Stage.IDLE) && (status.stage != Stage.ERROR);
    }

    /** True when the current volley has completed all 3 shots. */
    public boolean isDone() {
        return status.stage == Stage.DONE;
    }

    /** True if the volley encountered a recoverable error/timeout. */
    public boolean isError() {
        return status.stage == Stage.ERROR;
    }

    /** Get a snapshot of current status for telemetry. */
    public Status getStatus() {
        // Update fields that are cheap to refresh on access
        refreshSensedColors();
        refreshBusyFlags();
        refreshLauncherRpm();
        return status;
    }

    /** Call once per loop. Safe to call always; it will do nothing when IDLE. */
    public void update() {
        if (!active) {
            status.stage = Stage.IDLE;
            return;
        }

        // Keep commonly needed status fields updated
        refreshSensedColors();
        refreshBusyFlags();
        refreshLauncherRpm();

        // If we somehow got an empty desired sequence, bail
        if (desiredSequence.size() != 3) {
            status.stage = Stage.ERROR;
            status.debugNote = "bad motif sequence";
            return;
        }

        // If finished
        if (status.shotIndex >= 3) {
            status.stage = Stage.DONE;
            active = false;
            return;
        }

        // Desired for this shot
        status.desired = desiredSequence.get(status.shotIndex);

        switch (status.stage) {
            case POSITIONING_FOR_SHOT:
                // Ensure desired color is on LEFT or RIGHT (visible)
                if (isDesiredVisible()) {
                    status.spinnerAttempts = 0;
                    status.stage = Stage.FIRING_FLIPPER;
                    stateTimer.reset();
                } else {
                    // Not visible. Rotate spinner one step and wait.
                    rotateSpinnerOneStep(status.spinnerAttempts);
                    status.spinnerAttempts++;
                    status.stage = Stage.WAITING_FOR_SPINNER;
                    stateTimer.reset();
                }
                break;

            case WAITING_FOR_SPINNER:
                // Wait until spinner finishes or timeout
                if (!robot.isSpinnerBusy()) {
                    status.stage = Stage.POSITIONING_FOR_SHOT;
                    stateTimer.reset();
                } else if (stateTimer.milliseconds() > spinnerTimeoutMs) {
                    status.stage = Stage.ERROR;
                    status.debugNote = "spinner timeout";
                }
                break;

            case FIRING_FLIPPER:
                // Optional launcher ready gating
                if (requireLauncherReady && status.launcherRpm < launcherReadyRpm) {
                    // Stay here until ready (or timeout)
                    if (stateTimer.milliseconds() > 1500) {
                        // Don't hard-fail; allow firing anyway to keep auto moving
                        status.debugNote = "launcher not ready; firing anyway";
                    } else {
                        break;
                    }
                }

                // Decide side to fire based on which sensor matches desired color
                if (status.left == status.desired) {
                    robot.triggerLeftFlipper();
                } else if (status.right == status.desired) {
                    robot.triggerRightFlipper();
                } else {
                    // We lost the desired color while waiting. Go back to positioning.
                    status.stage = Stage.POSITIONING_FOR_SHOT;
                    stateTimer.reset();
                    break;
                }

                status.stage = Stage.WAITING_FOR_FLIPPER;
                stateTimer.reset();
                break;

            case WAITING_FOR_FLIPPER:
                if (!robot.areFlippersBusy()) {
                    // Advance to next shot
                    status.shotIndex++;
                    status.spinnerAttempts = 0;
                    status.stage = Stage.POSITIONING_FOR_SHOT;
                    stateTimer.reset();
                } else if (stateTimer.milliseconds() > flipperTimeoutMs) {
                    status.stage = Stage.ERROR;
                    status.debugNote = "flipper timeout";
                }
                break;

            case DONE:
            case IDLE:
            case ERROR:
            default:
                break;
        }

        // Guard: avoid infinite spinner attempts on a single shot
        if (status.stage == Stage.POSITIONING_FOR_SHOT && !isDesiredVisible()) {
            if (status.spinnerAttempts >= 3) {
                status.stage = Stage.ERROR;
                status.debugNote = "could not position desired color";
            }
        }
    }

    // ---------------- Tunables ----------------

    public void setLauncherReadyRpm(double rpm) { this.launcherReadyRpm = rpm; }
    public void setRequireLauncherReady(boolean require) { this.requireLauncherReady = require; }
    public void setSpinnerTimeoutMs(double ms) { this.spinnerTimeoutMs = ms; }
    public void setFlipperTimeoutMs(double ms) { this.flipperTimeoutMs = ms; }

    // ---------------- Helpers ----------------

    private void refreshSensedColors() {
        status.left = robot.getIntakeSlotColor(IntakeSensorFusion002.IntakeSlot.LEFT);
        status.right = robot.getIntakeSlotColor(IntakeSensorFusion002.IntakeSlot.RIGHT);
    }

    private void refreshBusyFlags() {
        status.spinnerBusy = robot.isSpinnerBusy();
        status.flippersBusy = robot.areFlippersBusy();
    }

    private void refreshLauncherRpm() {
        // Use left motor as representative (your code uses left for telemetry)
        if (robot.launcher != null) {
            status.launcherRpm = robot.launcher.getLeftMotorVelocityRPM();
        }
    }

    private boolean isDesiredVisible() {
        return status.left == status.desired || status.right == status.desired;
    }

    /**
     * Rotate the spinner one step.
     * We alternate directions each attempt to reduce “always rotating the long way”
     * when the mechanism has backlash or asymmetric indexing.
     */
    private void rotateSpinnerOneStep(int attempt) {
        if ((attempt % 2) == 0) robot.rotateSpinnerLeft();
        else robot.rotateSpinnerRight();
    }

    /** Converts motif string into a 3-color sequence. */
    private List<IntakeSensorFusion002.ArtifactColor> motifToSequence(String motif) {
        if (motif == null) return defaultSequence();

        String m = motif.trim().toUpperCase();
        if (m.length() != 3) return defaultSequence();

        List<IntakeSensorFusion002.ArtifactColor> seq = new ArrayList<>(3);
        for (int i = 0; i < 3; i++) {
            char c = m.charAt(i);
            if (c == 'G') seq.add(IntakeSensorFusion002.ArtifactColor.GREEN);
            else if (c == 'P') seq.add(IntakeSensorFusion002.ArtifactColor.PURPLE);
            else seq.add(IntakeSensorFusion002.ArtifactColor.UNKNOWN);
        }
        // If unknowns appear, fall back to safe default
        for (IntakeSensorFusion002.ArtifactColor c : seq) {
            if (c == IntakeSensorFusion002.ArtifactColor.UNKNOWN) return defaultSequence();
        }
        return seq;
    }

    private List<IntakeSensorFusion002.ArtifactColor> defaultSequence() {
        // Reasonable default: P,P,G
        List<IntakeSensorFusion002.ArtifactColor> seq = new ArrayList<>(3);
        seq.add(IntakeSensorFusion002.ArtifactColor.PURPLE);
        seq.add(IntakeSensorFusion002.ArtifactColor.PURPLE);
        seq.add(IntakeSensorFusion002.ArtifactColor.GREEN);
        return seq;
    }

    /**
     * Helper requested: deduce the third ball color given left/right.
     * Assumes: exactly 2 PURPLE + 1 GREEN total.
     *
     * If either sensor is UNKNOWN, we return UNKNOWN.
     */
    public static IntakeSensorFusion002.ArtifactColor deduceThirdBall(
            IntakeSensorFusion002.ArtifactColor left,
            IntakeSensorFusion002.ArtifactColor right
    ) {
        if (left == IntakeSensorFusion002.ArtifactColor.UNKNOWN || right == IntakeSensorFusion002.ArtifactColor.UNKNOWN) {
            return IntakeSensorFusion002.ArtifactColor.UNKNOWN;
        }
        if (left == IntakeSensorFusion002.ArtifactColor.GREEN && right == IntakeSensorFusion002.ArtifactColor.GREEN) {
            // Contradiction with the assumed set
            return IntakeSensorFusion002.ArtifactColor.UNKNOWN;
        }
        if (left == IntakeSensorFusion002.ArtifactColor.PURPLE && right == IntakeSensorFusion002.ArtifactColor.PURPLE) {
            return IntakeSensorFusion002.ArtifactColor.GREEN;
        }
        // One of each visible -> third must be PURPLE
        if ((left == IntakeSensorFusion002.ArtifactColor.GREEN && right == IntakeSensorFusion002.ArtifactColor.PURPLE) ||
                (left == IntakeSensorFusion002.ArtifactColor.PURPLE && right == IntakeSensorFusion002.ArtifactColor.GREEN)) {
            return IntakeSensorFusion002.ArtifactColor.PURPLE;
        }
        return IntakeSensorFusion002.ArtifactColor.UNKNOWN;
    }
}
