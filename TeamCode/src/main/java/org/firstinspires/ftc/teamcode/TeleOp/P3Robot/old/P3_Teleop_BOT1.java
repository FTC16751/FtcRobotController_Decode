/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.TeleOp.P3Robot.old;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utilities.P3Robot.P3_Robot_Bot1;

/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="P3: BOT1 Teleop (Team Version)", group=" _P3opmodes")
@Disabled
public class P3_Teleop_BOT1 extends OpMode
{
    public static final double TX_ALIGN_KP = 0.02;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private P3_Robot_Bot1 robot;


    enum allianceColor {
        RED,
        BLUE
    }
    private double requestedMotorVelocity = 1100;


    // TODO: set this at init
    allianceColor alliance = allianceColor.RED;

    private enum IntakeState {
        ON,
        OFF,
        REVERSE;
    }
    private IntakeState intakeState = IntakeState.OFF;
    private final double INTAKE_POWER = 1.0;
    double txError = 0.0;
    double tagYaw =0.0;
    double angleOnTarget = 0.0;
    public final double TX_ALIGN_TOLERANCE_DEG = 1.0;
    public final double SHOOTER_VELOCITY_TOLERANCE_PERCENT = 0.95;
    public static final double TURRET_RIGHT_ANGLE = 90.0;
    public static final double TURRET_LEFT_ANGLE = -90.0;
    public static final double TURRET_BACK_ANGLE = -180.0; // Note: -180 and 180 are the same position.
    public static final double TURRET_HOME_ANGLE = 0.0;

    public static final double TURRET_MANUAL_ADJUST_DEGREES = 5.0;
    public static final double JOYSTICK_DEADBAND = 0.05; // Increased slightly for robustness

    // In P3_Teleop_BOT1.java, with your other member variables

    // --- NEW: State for Turret Control ---
    private enum TurretControlMode {
        ANGLE_CONTROL, // When using setTargetAngle()
        POWER_CONTROL  // When using the joystick for direct power
    }
    private TurretControlMode turretControlMode = TurretControlMode.ANGLE_CONTROL; // Default to angle control


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot = new P3_Robot_Bot1(hardwareMap,telemetry);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        robot.update();
       // doDriveControls();
        handleIntakeControls();
//        calcShooterVelocity();
        handleLauncherControls();
        handleTurret();


//        telemetry.addData("Left Front motor position: ", robot.drive.getmotorPosition(robot.drive.leftFrontMotor));
//        telemetry.addData("Left Rearmotor position: ", robot.drive.getmotorPosition(robot.drive.leftRearMotor));
//        telemetry.addData("Right Front motor position: ", robot.drive.getmotorPosition(robot.drive.rightFrontMotor));
//        telemetry.addData("Right Rear motor position: ", robot.drive.getmotorPosition(robot.drive.rightRearMotor));
//        telemetry.addData("current X coordinate", robot.drive.getOdoPosition().getX(DistanceUnit.INCH));
//        telemetry.addData("current Y coordinate", robot.drive.getOdoPosition().getY(DistanceUnit.INCH));
//        telemetry.addData("current Heading angle", robot.drive.getOdoPosition().getHeading(AngleUnit.DEGREES));
        telemetry.addData("requested motor velocity: ", requestedMotorVelocity);
        telemetry.addData("actual motor velocity: ", robot.launcher.getShooterMotorVelocity());
        telemetry.addData("tx error: ", txError);
        // Turret information
        robot.turret.addTelemetry(telemetry);
        telemetry.addLine();

        // Control hints
        telemetry.addData("turretControlMode: ", turretControlMode);
        telemetry.addData("Right Stick X", "%.2f", gamepad1.right_stick_x);
        telemetry.addLine("─────────────────────");
        telemetry.addLine("Controls:");
        telemetry.addLine("  Right Stick X = Rotate");
        telemetry.addLine("  A = Home Position");
        telemetry.addLine("  B = Emergency Stop");
        telemetry.addLine("  D-pad L/R = Fine Adjust ±5°");
        telemetry.update();

    }

    private void handleTurret() {
        // --- Input Detection ---
        boolean isAngleButtonJustPressed = gamepad2.bWasPressed() || gamepad2.xWasPressed() ||
                gamepad2.yWasPressed() || gamepad2.aWasPressed() ||
                gamepad2.dpadLeftWasPressed() || gamepad2.dpadRightWasPressed();

        boolean isJoystickActive = Math.abs(gamepad2.right_stick_x) > JOYSTICK_DEADBAND;

        // --- State Switching Logic ---
        // If the joystick is moved, switch to POWER_CONTROL mode.
        if (isJoystickActive) {
            turretControlMode = TurretControlMode.POWER_CONTROL;
        }
        // If any angle button is pressed, switch to ANGLE_CONTROL mode.
        else if (isAngleButtonJustPressed) {
            turretControlMode = TurretControlMode.ANGLE_CONTROL;
        }

        // --- State Action Logic ---
        switch (turretControlMode) {
            case ANGLE_CONTROL:
                telemetry.addData("in angle contrul: ", " pressed a button");

                // In this mode, we only listen for button presses that set a target angle.
                // We do NOT send any power commands.
                if (gamepad2.bWasReleased()) {
                    telemetry.addData("b button: ", " was pressed");
                    robot.turret.setTargetAngle(TURRET_RIGHT_ANGLE);
                } else if (gamepad2.xWasReleased()) {
                    telemetry.addData("x button: ", " was pressed");

                    robot.turret.setTargetAngle(TURRET_LEFT_ANGLE);
                } else if (gamepad2.y) {
                    telemetry.addData("y button: ", " was pressed");

                    robot.turret.returnToHome();
                } else if (gamepad2.aWasReleased()) {
                    robot.turret.setTargetAngle(TURRET_BACK_ANGLE);
                } else if (gamepad2.dpadLeftWasReleased()) {
                    robot.turret.rotateRelative(-TURRET_MANUAL_ADJUST_DEGREES);
                } else if (gamepad2.dpadRightWasReleased()) {
                    robot.turret.rotateRelative(TURRET_MANUAL_ADJUST_DEGREES);
                }
                // When in ANGLE_CONTROL and no buttons are pressed, we do nothing.
                // This allows the motor's RUN_TO_POSITION controller to continue
                // working without interference.
                break;

            case POWER_CONTROL:
                // In this mode, we only listen to the joystick.
                // The setTurretPower method should handle switching the motor mode
                // to RUN_USING_ENCODER or RUN_WITHOUT_ENCODER.
                if (isJoystickActive) {
                    robot.turret.setTurretPower(gamepad2.right_stick_x);
                } else {
                    // Joystick has returned to center. Stop manual power control.
                    robot.turret.setTurretPower(0);
                    // IMPORTANT: We should now tell the turret to hold its current position
                    // using its angle controller. This prevents it from drifting.
                    robot.turret.holdPosition();
                    turretControlMode = TurretControlMode.ANGLE_CONTROL; // Return to default mode
                }
                break;
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
    private void doDriveControls() {
        //double turnInput = 0;
        double driveInput  = gamepad1.left_stick_y;
        double strafeInput = gamepad1.left_stick_x;
        double turnInput = gamepad1.right_stick_x;
        boolean isSnappingToTarget = gamepad1.right_stick_button && robot.vision.isTargetVisible();
        double txError = robot.vision.getTargetAngleX();;
        
        if (Math.abs(txError) <= TX_ALIGN_TOLERANCE_DEG) {
            angleOnTarget = 0.0;
        } else {
            angleOnTarget = TX_ALIGN_KP * txError;
        }
        if (isSnappingToTarget) {
            turnInput = angleOnTarget;
            telemetry.addData("TX Align", "ON | Error: %.1f deg", txError);
        } else {
            // normal right-stick turning
            turnInput = gamepad1.right_stick_x;
        }



        robot.drive.arcadeDrive(-strafeInput, driveInput, turnInput, gamepad1.right_stick_y, 1.0);

        //drive.arcadeDrive(strafeInput, driveInput, turnInput, gamepad1.right_stick_y, DRIVE_SPEED);

    }

    private void handleIntakeControls() {
        if (gamepad1.aWasPressed()) {
            intakeState = (intakeState == IntakeState.ON) ? IntakeState.OFF : IntakeState.ON;
        }
        switch (intakeState) {
            case ON:
                robot.intake.setIntakeMotor(-INTAKE_POWER);
                robot.intake.setIntakeServos();
                break;
            case OFF:
                robot.intake.setIntakePower(0.0);
                robot.intake.stopIntakeServos();
                break;

        }
    }
/**
 * @return the velocity of the shooter motor in ticks per second, based on distance to target
    */
    
    private double calcShooterVelocity() {
        double h = 0.9845-0.381;
        double angle = Math.toRadians(55);
        double distance = robot.vision.getDistanceToTagMeters();
        telemetry.addData("distance ", distance);
        double flyWheelCircumference = 0.09525*Math.PI;
        double velocity = Math.sqrt(Math.pow(9.8*distance, 2)/
                (2*Math.cos(angle)*Math.cos(angle))
                * (distance*Math.tan(angle)) - h);
        telemetry.addData("velocity ", velocity);
        double rotations = velocity / flyWheelCircumference;
        double ticks;
        ticks = 28 * rotations;
        return ticks;
    }

    private void handleLauncherControls() {
        if (gamepad1.right_trigger > 0.8) {
            robot.launcher.setIndexerServoPower(-1.0);
            robot.launcher.setShootingPosition();
        }
        else {
            robot.launcher.setIndexerServoPower(0.0);
            robot.launcher.setStopPosition();
        }

        if (gamepad1.yWasPressed()) {
            requestedMotorVelocity = calcShooterVelocity();
            robot.launcher.setShooterMotorVelocity(1300);
            // CHANGE THIS BACK TO CALC SHOOTER VELOCITY LATER
        }
        if (gamepad1.xWasPressed()) {
            robot.launcher.setShooterMotorVelocity(0);
        }

        if (gamepad1.dpadUpWasPressed()) {
            requestedMotorVelocity = requestedMotorVelocity+100;
            robot.launcher.setShooterMotorVelocity(requestedMotorVelocity);

        } else if (gamepad1.dpadDownWasPressed()){
            requestedMotorVelocity = requestedMotorVelocity-100;
            robot.launcher.setShooterMotorVelocity(requestedMotorVelocity);

        } else if (gamepad1.startWasPressed()){
            requestedMotorVelocity = 1200;
            robot.launcher.setShooterMotorVelocity(requestedMotorVelocity);

        }
    }

}
