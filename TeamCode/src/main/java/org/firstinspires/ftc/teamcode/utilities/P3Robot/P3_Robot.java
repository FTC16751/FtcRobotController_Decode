package org.firstinspires.ftc.teamcode.utilities.P3Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.Common.DriveUtil2026;
import org.firstinspires.ftc.teamcode.utilities.Common.InterpolatingLookupTable;
import org.firstinspires.ftc.teamcode.utilities.Common.LimeLightVisionUtil;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.IntakeSensorFusion;

/**
 * P3_Robot is the central hub that orchestrates all of the P3 robot's subsystems.
 * It owns all the hardware and utility classes, providing a clean interface for OpModes.
 */
public class P3_Robot {

    // --- PUBLIC SUBSYSTEMS ---
    // These are public so the OpMode can access them directly (e.g., robot.drive.arcadeDrive(...))
    public final DriveUtil2026 drive;
    public final P3_IntakeUtil intake;
    public final P3_LauncherUtil launcher;
    public final LimeLightVisionUtil vision;
    public final Telemetry telemetry;
    private InterpolatingLookupTable flywheelTable;
    /**
     * Constructor for the P3_Robot class.
     */
    public P3_Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Initialize all subsystems
        // We pass 'opMode' itself, which contains hardwareMap and telemetry
        drive = new DriveUtil2026(hardwareMap, telemetry, null);
        intake = new P3_IntakeUtil(hardwareMap);
        launcher = new P3_LauncherUtil(hardwareMap);
        vision = new LimeLightVisionUtil(hardwareMap, telemetry);

        flywheelTable = new InterpolatingLookupTable();
        flywheelTable.add(30.0, 700.0);
        flywheelTable.add(40.0, 750.0);
        flywheelTable.add(50.0, 820.0);
        flywheelTable.add(60.0, 850.0);
        flywheelTable.add(70.0, 920.0);
        flywheelTable.add(80.0, 980.0);
        flywheelTable.add(100.0, 1090.0);
        flywheelTable.add(120.0, 1120.0);
        flywheelTable.add(130.0, 1370.0);
        flywheelTable.add(140.0, 1175.0);
    }

    /**
     * The main periodic update method for the robot.
     * This MUST be called in every iteration of the OpMode's loop().
     */
    public void update() {
        // In the future, any subsystems that need continuous updates would be called here.
        // For now, it's a placeholder.
        // e.g., drive.update();
        vision.update();
    }

    /**
     * Stops all motors and mechanisms on the robot. Call this in the OpMode's stop() method.
     */
    public void stopAll() {
        drive.stopRobot(); // Assuming a method like this exists in DriveUtil2026
        intake.setIntakeMotorPower(0);
        launcher.setShooterMotorVelocity(0);
        vision.stop();
    }
    public double getTargetVelocityForDistance(double distanceInches) {
        // This method safely accesses the private flywheelTable.
        return flywheelTable.get(distanceInches);
    }
    /**
     * A consolidated method for displaying common robot telemetry.
     */
    public void addTelemetry() {
        telemetry.addLine("--- P3 Robot Telemetry ---");
        // Add telemetry from subsystems
        vision.addTelemetry();
        // You can add more telemetry from other subsystems here
        // e.g., telemetry.addData("Shooter Velocity", launcher.getShooterMotorVelocity());
    }
}
