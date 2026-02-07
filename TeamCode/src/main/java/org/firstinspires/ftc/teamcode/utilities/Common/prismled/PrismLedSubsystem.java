package org.firstinspires.ftc.teamcode.utilities.Common.prismled;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PrismLedSubsystem {

    public enum Mode {
        DRIVE,
        ENDGAME,
        IDLE
    }

    private final GoBildaPrismDriver prism;
    private final ElapsedTime runtime = new ElapsedTime();

    private Mode baseMode = Mode.DRIVE;
    private double flashUntil = 0;

    private GoBildaPrismDriver.Artboard lastArtboard = null;

    public PrismLedSubsystem(GoBildaPrismDriver prism) {
        this.prism = prism;
        runtime.reset();
    }

    /* ================== EVENTS FROM OPMODE ================== */

    public void onTeleOpStart() {
        runtime.reset();
        baseMode = Mode.DRIVE;
    }

    public void onMatchEnd() {
        baseMode = Mode.IDLE;
    }

    public void onShotFired() {
        flashUntil = runtime.seconds() + 0.25; // 250ms
    }

    public void onEndgame() {
        baseMode = Mode.ENDGAME;
    }

    /* ================== PERIODIC UPDATE ================== */

    public void update() {
        GoBildaPrismDriver.Artboard desired;

        if (baseMode == Mode.IDLE) {
            desired = GoBildaPrismDriver.Artboard.ARTBOARD_3; // faint blue
        } else if (baseMode == Mode.ENDGAME) {
            desired = GoBildaPrismDriver.Artboard.ARTBOARD_2; // heartbeat red
        } else if (runtime.seconds() < flashUntil) {
            desired = GoBildaPrismDriver.Artboard.ARTBOARD_1; // shoot flash
        } else {
            desired = GoBildaPrismDriver.Artboard.ARTBOARD_0; // drive blend rotate
        }

        if (desired != lastArtboard) {
            prism.loadAnimationsFromArtboard(desired);
            lastArtboard = desired;
        }
    }
}

