package org.firstinspires.ftc.teamcode.utilities.PriorSeason;

import static org.firstinspires.ftc.teamcode.utilities.PriorSeason.Servo_Helper_Functions.IntakeRotateAngle.ONE_EIGHTY;
import static org.firstinspires.ftc.teamcode.utilities.PriorSeason.Servo_Helper_Functions.IntakeRotateAngle.ZERO;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.Common.DriveUtil2025;

public class Robot {
    // --- Subsystem Instances ---
    public DriveUtil2025 drive;
    public Horizontal_Slide_Helper_Functions horizontalSlides;
    public Vertical_Slide_Helper_Functions verticalSlides;
    public Servo_Helper_Functions claw;
    public Slide_Hang_Helper_Functions linearActuator;
    public Telemetry telemetry;
    public TouchSensor beamBreak;
    // State Machine Enums
    public enum ArmTransferState {
        INITIAL,
        SLIDE_EXTENDING,
        SNAPPED,
        WAITING_FOR_CLAW_CLOSE,
        CLAW_CLOSED,
        ELBOW_RAISED,
        WRIST_RAISED,
        SLIDE_RETRACTING,
        PARTIAL_SNAPPING,
        PARTIAL_WAITING_FOR_CLAW_CLOSE,
        PARTIAL_CLAW_CLOSED,
        PARTIAL_ARM_RETRACTING,
        WRIST_EXTEND, WAIT, DONE
    }

    public enum ClimberState {
        IDLE,
        RAISING,
        LOWERING_INIT,
        LOWERING_SLIDES,
        LOWERING_WAIT_FOR_SLIDES,
        LOWERING_RAISE_STOPPER,
        LOWERING_WAIT_FOR_STOPPER,
        LOWERING_FINAL
    }
    public enum ScoringState {
        CLOSE_LIFT_CLAW,
        WAIT_1,
        OPEN_INTAKE_CLAW,
        WAIT_2,
        MOVE_ARM_FOR_SCORING_LIFT,
        MOVE_ELBOW_TO_SCORE,
        MOVE_WRIST_TO_SCORE,
        RAISE_VERTICAL_SLIDES,
        DONE
    }

    // --- Constants ---
    private static final double SLIDE_EXTEND_DELAY = 0.3;
    private static final double ARM_TRANSFER_WAIT_TIME = 0.5;
    private static final double CLAW_CLOSE_DELAY = 0.3;
    private static final double WAIT_TIME = 0.5;
    private static final double LOWERING_SLIDES_DELAY = 1.0;
    private static final double LOWERING_STOPPER_DELAY = 0.5;
    // --- State Variables ---
    private ArmTransferState armTransferState = ArmTransferState.INITIAL;
    private ClimberState climberState = ClimberState.IDLE;
    private ScoringState scoringState = ScoringState.DONE;
    private boolean partialSnapPerformed = false;

    // --- Timers ---
    private ElapsedTime waitTimer = new ElapsedTime();
    private ElapsedTime clawCloseTimer = new ElapsedTime();
    private ElapsedTime slideExtendTimer = new ElapsedTime();
    private ElapsedTime climberTimer = new ElapsedTime();


    // --- Constructor ---
    public Robot(HardwareMap hardwareMap, Telemetry telemetry) throws InterruptedException {
        this.telemetry = telemetry;
        drive = new DriveUtil2025(null); // Pass null for the OpMode context here
        drive.init(hardwareMap, telemetry);
        horizontalSlides = new Horizontal_Slide_Helper_Functions(hardwareMap, telemetry);
        verticalSlides = new Vertical_Slide_Helper_Functions(hardwareMap, telemetry);
        claw = new Servo_Helper_Functions(hardwareMap);
        linearActuator = new Slide_Hang_Helper_Functions(hardwareMap, telemetry);
        beamBreak = hardwareMap.get(TouchSensor.class, "beambreak");
    }

    // --- Drivetrain Methods ---
    public void arcadeDrive(double x, double y, double turn, double speed) {
        drive.arcadeDrive(x, y, turn, turn, speed);
    }

    // --- Horizontal Slide Methods ---
    public void moveHorizontalSlidesToLevelZero() {
        horizontalSlides.setCurrentState(Horizontal_Slide_Helper_Functions.SlideState.LEVEL_ZERO);
        horizontalSlides.updateStateMachine();
    }

    public void moveHorizontalSlidesToLowPosition() {
        horizontalSlides.setCurrentState(Horizontal_Slide_Helper_Functions.SlideState.LOW_POSITION);
        horizontalSlides.updateStateMachine();
    }

    public void decreaseHorizontalSlidePosition(double increment) {
        if (!horizontalSlides.isTouchSensorPressed()) {
            horizontalSlides.decreasePosition(increment);
        }
    }

    public void increaseHorizontalSlidePosition(double increment) {
        horizontalSlides.increasePosition(increment);
    }

    public boolean isHorizontalSlideTouchSensorPressed() {
        return horizontalSlides.isTouchSensorPressed();
    }

    // --- Vertical Slide Methods ---
    public void moveVerticalSlidesToLevelZero() {
        verticalSlides.setCurrentState(Vertical_Slide_Helper_Functions.SlideState.LEVEL_ZERO);
        verticalSlides.updateStateMachine();
    }

    public void moveVerticalSlidesToLowPosition() {
        verticalSlides.setCurrentState(Vertical_Slide_Helper_Functions.SlideState.LOW_POSITION);
        verticalSlides.updateStateMachine();
    }

    public void decreaseVerticalSlidePosition(int increment) {
        if (!verticalSlides.isTouchSensorPressed()) {
            verticalSlides.decreasePosition(increment);
        } else if (verticalSlides.isTouchSensorPressed()) {
            verticalSlides.resetEncoder();
        }
    }

    public void increaseVerticalSlidePosition(int increment) {
        verticalSlides.increasePosition(increment);
    }

    public boolean isVerticalSlideTouchSensorPressed() {
        return verticalSlides.isTouchSensorPressed();
    }

    public void stopVerticalSlides() {
        verticalSlides.stopSlides();
        verticalSlides.resetEncoder();
    }

    public void moveVerticalSlidesToPosition(int position) {
        verticalSlides.moveToPosition(position);
    }

    public boolean areVerticalSlidesAtTargetPosition(Vertical_Slide_Helper_Functions.SlideState state) {
        return verticalSlides.slidesAtTargetPosition(verticalSlides.getTargetPosition(state));
    }

    // --- Claw Methods ---
    public void toggleIntakeClaw() {
        claw.toggleIntakeClaw();
    }

    public void toggleLiftClaw() {
        claw.toggleLiftClaw();
    }

    public void adjustRotateClaw(double increment) {
        claw.adjustRotateClaw(increment);
    }

//    public void rotateIntakeClaw(Servo_Helper_Functions.IntakeRotateAngle angle) {
//        claw.rotateIntakeClaw(angle);
//    }
    // New method in Robot to handle the rotation
    public void rotateIntakeClaw(Servo_Helper_Functions.IntakeRotateAngle angle) {
        Servo_Helper_Functions.IntakeRotateAngle servoAngle;
        switch (angle) {
            case ZERO:
                servoAngle = Servo_Helper_Functions.IntakeRotateAngle.ZERO;
                break;
            case FORTY_FIVE:
                servoAngle = Servo_Helper_Functions.IntakeRotateAngle.FORTY_FIVE;
                break;
            case NINETY:
                servoAngle = Servo_Helper_Functions.IntakeRotateAngle.NINETY;
                break;
            case ONE_THIRTY_FIVE:
                servoAngle = Servo_Helper_Functions.IntakeRotateAngle.ONE_THIRTY_FIVE;
                break;
            case ONE_EIGHTY:
                servoAngle = ONE_EIGHTY;
                break;
            default:
                servoAngle = Servo_Helper_Functions.IntakeRotateAngle.ZERO;
                break;
        }
        claw.rotateIntakeClaw(servoAngle);
    }
    public void adjustIntakeElbowPosition(double increment) {
        claw.adjustIntakeElbowPosition(increment);
    }

    public void incrementIntakeWrist(double increment) {
        claw.incrementIntakeWrist(increment);
    }

    public void adjustLiftElbowPosition(double increment) {
        claw.adjustLiftElbowPosition(increment);
    }

    public void moveArmToAcquireGamePiece() {
        claw.moveArmToAcquireGamePiece();
    }

    public void snapGamePiece() {
        claw.snapGamePiece();
    }

    public void closeIntakeClaw() {
        claw.closeIntakeClaw();
    }

    public void openIntakeClaw() {
        claw.openIntakeClaw();
    }

    public void raiseIntakeElbow() {
        claw.raiseIntakeElbow();
    }

    public void raiseIntakeWrist() {
        claw.raiseIntakeWrist();
    }

    public void lowerIntakeWrist() {
        claw.lowerIntakeWrist();
    }

    public void moveLiftWristServotoTransferPosition() {
        claw.moveLiftWristServotoTransferPosition();
    }

    public void moveLiftElbowServotoTransferPosition() {
        claw.moveLiftElbowServotoTransferPosition();
    }

    public void moveLiftArmforGettingSpecimen() {
        claw.moveLiftArmforGettingSpecimen();
    }

    public void moveLiftArmforScoringSpecimen() {
        claw.moveLiftArmforScoringSpecimen();
    }

    public void moveArmforScoringLift() {
        claw.moveArmforScoringLift();
    }

    public void moveLiftElbowServotoScorePosition() {
        claw.moveLiftElbowServotoScorePosition();
    }

    public void moveLiftWristServotoScorePosition() {
        claw.moveLiftWristServotoScorePosition();
    }

    public void moveIntakeArmToTransferGamePiece() {
        claw.moveIntakeArmToTransferGamePiece();
    }

    public void closeLiftClaw() {
        claw.closeLiftClaw();
    }

    public void openLiftClaw() {
        claw.openLiftClaw();
    }

    public void raiseSlideStopper() {
        claw.raiseSlideStopper();
    }

    public void lowerSlideStopper() {
        claw.lowerSlideStopper();
    }

    // --- Linear Actuator Methods ---
    public void increaseLinearActuatorPosition(int increment) {
        linearActuator.increasePosition(increment);
    }

    public void decreaseLinearActuatorPosition(int increment) {
        linearActuator.decreasePosition(increment);
    }

    public void setLinearActuatorState(Slide_Hang_Helper_Functions.LiftState state) {
        linearActuator.setCurrentState(state);
        linearActuator.updateStateMachine();
    }

    // --- Telemetry Methods ---
    public void addTelemetry() {
        verticalSlides.sendTelemetryData();
        horizontalSlides.sendTelemetryData();
        telemetry.addData("Horizontal Slide reported current State ", horizontalSlides.getCurrentState());
        telemetry.addData("Left Slide getPosition ", horizontalSlides.getLeftSlidePosition());
        telemetry.addData("Left Slide getPosition ", horizontalSlides.getRightSlidePosition());
        telemetry.addData("horizontal touch sensor: ", horizontalSlides.isTouchSensorPressed());
        telemetry.addLine();
        telemetry.addData("Vertical Slide reported current State ", verticalSlides.getCurrentState());
        telemetry.addData("Vertical Slide getTargetPosition ", verticalSlides.getTargetPosition(verticalSlides.getCurrentState()));
        telemetry.addData("Vertical Slide slidesAtTargetPosition ", verticalSlides.slidesAtTargetPosition(verticalSlides.getTargetPosition(verticalSlides.getCurrentState())));
        telemetry.addData("Vertical touch sensor: ", verticalSlides.isTouchSensorPressed());
    }

    // State Machine Methods
    public void updateArmTransfer() {
        switch (armTransferState) {
            case INITIAL:
                // Do nothing, wait for button press
                break;
            case SLIDE_EXTENDING:
                openIntakeClaw();
                moveArmToAcquireGamePiece();
                lowerIntakeWrist();
                slideExtendTimer.reset();
                armTransferState = ArmTransferState.WRIST_EXTEND;
                break;
            case WRIST_EXTEND:
                moveHorizontalSlidesToLowPosition();
                armTransferState = ArmTransferState.DONE;
                break;
            case SNAPPED:
                snapGamePiece();
                clawCloseTimer.reset();
                armTransferState = ArmTransferState.WAITING_FOR_CLAW_CLOSE;
                break;
            case WAITING_FOR_CLAW_CLOSE:
                if (clawCloseTimer.seconds() >= CLAW_CLOSE_DELAY) {
                    closeIntakeClaw();
                    armTransferState = ArmTransferState.CLAW_CLOSED;
                }
                break;
            case CLAW_CLOSED:
                waitTimer.reset();
                armTransferState = ArmTransferState.ELBOW_RAISED;
                break;
            case ELBOW_RAISED:
                if (waitTimer.seconds() >= ARM_TRANSFER_WAIT_TIME || partialSnapPerformed) {
                    raiseIntakeElbow();
                    armTransferState = ArmTransferState.WAIT;
                    waitTimer.reset();
                }
                break;
            case WAIT:
                if (waitTimer.seconds() >= ARM_TRANSFER_WAIT_TIME) {
                    armTransferState = ArmTransferState.SLIDE_RETRACTING;
                }
                break;
            case SLIDE_RETRACTING:
                moveHorizontalSlidesToLevelZero();
                armTransferState = ArmTransferState.WRIST_RAISED; // Move to next state
                break;
            case WRIST_RAISED:
                raiseIntakeWrist();
                rotateIntakeClaw(ZERO);
                moveLiftWristServotoTransferPosition();
                armTransferState = ArmTransferState.DONE;
                break;
            case PARTIAL_SNAPPING:
                partialSnapPerformed = true;
                snapGamePiece();
                clawCloseTimer.reset();
                armTransferState = ArmTransferState.PARTIAL_WAITING_FOR_CLAW_CLOSE;
                break;
            case PARTIAL_WAITING_FOR_CLAW_CLOSE:
                if (clawCloseTimer.seconds() >= CLAW_CLOSE_DELAY) {
                    closeIntakeClaw();
                    armTransferState = ArmTransferState.PARTIAL_CLAW_CLOSED;
                }
                break;
            case PARTIAL_CLAW_CLOSED:
                waitTimer.reset();
                armTransferState = ArmTransferState.PARTIAL_ARM_RETRACTING;
                break;
            case PARTIAL_ARM_RETRACTING:
                if (waitTimer.seconds() >= ARM_TRANSFER_WAIT_TIME) {
                    moveIntakeArmToTransferGamePiece();
                    armTransferState = ArmTransferState.DONE;
                }
                break;
            case DONE:
                // Do nothing, wait for button press
                break;
        }
    }
    public void updateClimber() {
        switch (climberState) {
            case IDLE:
                //do nothing, this is handled in the IDLE state
                break;
            case LOWERING_INIT:
                climberState = ClimberState.LOWERING_SLIDES;
                horizontalSlides.setCurrentState(Horizontal_Slide_Helper_Functions.SlideState.LEVEL_ZERO);
                horizontalSlides.updateStateMachine();
                claw.moveIntakeArmToTransferGamePiece();
                climberTimer.reset();
                break;
            case LOWERING_SLIDES:
                if (climberTimer.seconds() >= LOWERING_SLIDES_DELAY) {
                    climberState = ClimberState.LOWERING_RAISE_STOPPER;
                    claw.raiseSlideStopper();
                    climberTimer.reset();
                }
                break;
            case LOWERING_RAISE_STOPPER:
                if (climberTimer.seconds() >= LOWERING_STOPPER_DELAY) {
                    climberState = ClimberState.LOWERING_FINAL;
                    linearActuator.setCurrentState(Slide_Hang_Helper_Functions.LiftState.LOW_POSITION);
                    linearActuator.updateStateMachine();
                }
                break;
            case LOWERING_FINAL:
                climberState = ClimberState.IDLE;
                break;
            case RAISING:
                //do nothing, this is handled in the IDLE state
                break;
        }
    }

    public void updateScoring() {
        switch (scoringState) {
            case CLOSE_LIFT_CLAW:
                closeLiftClaw();
                waitTimer.reset();
                scoringState = ScoringState.WAIT_1;
                break;
            case WAIT_1:
                if (waitTimer.seconds() >= WAIT_TIME) {
                    scoringState = ScoringState.OPEN_INTAKE_CLAW;
                }
                break;
            case OPEN_INTAKE_CLAW:
                openIntakeClaw();
                waitTimer.reset();
                scoringState = ScoringState.WAIT_2;
                break;
            case WAIT_2:
                if (waitTimer.seconds() >= WAIT_TIME) {
                    scoringState = ScoringState.MOVE_ARM_FOR_SCORING_LIFT;
                }
                break;
            case MOVE_ARM_FOR_SCORING_LIFT:
                moveArmforScoringLift();
                scoringState = ScoringState.MOVE_ELBOW_TO_SCORE;
                break;
            case MOVE_ELBOW_TO_SCORE:
                moveLiftElbowServotoScorePosition();
                scoringState = ScoringState.MOVE_WRIST_TO_SCORE;
                break;
            case MOVE_WRIST_TO_SCORE:
                moveLiftWristServotoScorePosition();
                scoringState = ScoringState.RAISE_VERTICAL_SLIDES;
                break;
            case RAISE_VERTICAL_SLIDES:
                moveVerticalSlidesToLowPosition();
                scoringState = ScoringState.DONE;
                break;
            case DONE:
                // Do nothing, wait for button press
                break;
        }
    }

    // Public methods to start state machine sequences
    public void startArmTransferToAcquire() {
        armTransferState = ArmTransferState.SLIDE_EXTENDING;
    }

    public void startArmTransferToPartialSnap() {
        armTransferState = ArmTransferState.PARTIAL_SNAPPING;
    }

    public void startArmTransferToSnap() {
        armTransferState = ArmTransferState.SNAPPED;
    }

    public void startClimbLowering() {
        climberState = ClimberState.LOWERING_INIT;
    }

    public void startScoringSequence() {
        scoringState = ScoringState.CLOSE_LIFT_CLAW;
    }
}

