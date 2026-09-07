package org.firstinspires.ftc.teamcode.teams.testteam2027;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.DriveUtil2026b;
import org.firstinspires.ftc.teamcode.common.RobotConfig;
import org.firstinspires.ftc.teamcode.common.VisionUtil;

/**
 * test2027bot: the one object every OpMode for this robot creates. It owns the drive and the
 * camera and knows how to update them each loop. Subsystems for a real game (launcher, intake,
 * arm) get added here as public fields, built from device names in Test2027BotConfig.
 *
 * Rules that keep OpModes simple:
 *   - OpModes call robot.update() first thing in every loop() and init_loop().
 *   - OpModes call robot.stopAll() in stop().
 *   - OpModes never touch hardwareMap themselves.
 */
public class Test2027Robot {

    public final RobotConfig config;
    public final DriveUtil2026b drive;
    public final VisionUtil vision;       // null-safe inside: reports nothing if the Limelight is absent
    public final Telemetry telemetry;

    public Test2027Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.config = Test2027BotConfig.create();

        drive  = new DriveUtil2026b(hardwareMap, telemetry, null, config);
        vision = new VisionUtil(hardwareMap, telemetry, config.hardware.limelight);

        // How the robot operates comes from Constants; the beginner commands read these defaults.
        drive.setDefaultSpeeds(Test2027Constants.Drive.AUTO_DRIVE_SPEED, Test2027Constants.Drive.AUTO_TURN_SPEED);
        drive.setDefaultHoldTime(Test2027Constants.Auto.HOLD_SEC);
    }

    /** Point the camera's pipeline at this tag (goal tags and motif tags live on different pipelines). */
    public void lookForTag(int tagId) {
        vision.selectPipelineForTag(tagId);
    }

    /** Call in every loop() and init_loop(). Steps the Pinpoint, any async drive, and the camera. */
    public void update() {
        vision.update();     // camera first, so this loop's drive step sees this loop's tag
        drive.update();
    }

    public void stopAll() {
        drive.cancel();
        drive.stop();
        vision.stop();
    }

    /** The standard telemetry footer for this robot. */
    public void addTelemetry() {
        drive.addTelemetry();
        drive.getTagApproach().addTelemetry(telemetry);
        if (vision.isTargetVisible()) {
            int seen = vision.getDetectedTagId();
            if (vision.canSee(seen)) {
                telemetry.addData("sighting", "id %d  fwd %.1f in  right %.1f in  square %.1f deg",
                        seen, vision.forwardInches(), vision.rightInches(), vision.squareUpDegrees());
                telemetry.addData("raw", vision.sightedTagRaw());
            } else {
                telemetry.addData("sighting", "id %d seen, but the Limelight gave no robot-space pose (pipeline 3D setting)", seen);
            }
        } else {
            telemetry.addData("sighting", "no tag in view");
        }
    }
}
