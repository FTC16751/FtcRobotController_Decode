package org.firstinspires.ftc.teamcode.utilities.PriorSeason;

import static java.lang.Thread.sleep;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Servo_Helper_Functions {

    private Servo
            intakeClawServo,
            intakeRotateServo,
            intakeWristServo,
            leftElbowServo,
            rightElbowServo;
    private  Servo
            liftClawServo,
            liftWristServo,
            liftElbowServo,liftElbowServoR;

    // Constants for servo names. these are the ones that go into the configuration on the driver hub
    private static final String
            INTAKE_CLAW_SERVO_NAME = "intakeclaw",
            INTAKE_ROTATE_SERVO_NAME = "intakepivot",
            INTAKE_WRIST_SERVO_NAME = "intakewrist",
            INTAKE_ELBOW_RIGHT_SERVO_NAME = "intakeelbowright",
            INTAKE_ELBOW_LEFT_SERVO_NAME = "intakeelbowleft";
    private static final String
            LIFT_CLAW_SERVO_NAME = "liftclaw",
            LIFT_ELBOW_SERVO_NAME = "liftelbow",
            LIFT_ELBOWR_SERVO_NAME = "liftelbowR",
            LIFT_WRIST_SERVO_NAME = "liftwrist";

    public void toggleIntakeClaw() {
        if (isIntakeClawClosed) {
            openIntakeClaw();
        } else {
            closeIntakeClaw();
        }
    }

    public void toggleLiftClaw() {
        if (isLiftClawClosed) {
            openLiftClaw();
        } else {
            closeLiftClaw();
        }
    }



    //DEFINE SET POSITIONS FOR THE INTAKE SERVOS (claw, pivot, wrist, elbows)

    //claw servo positions
    public enum IntakeClawPosition {
        OPEN(0.8),
        CLOSED(1.0); // Replace 0.0 with the actual closed position

        public final double position;

        IntakeClawPosition(double servoPosition) {
            this.position = servoPosition;
        }
    }

    // Represents the positions of the intake wrist servo.
    public enum IntakeWristPosition {
        INIT(0.0),
        RAISE(0.26),
        SPECIMEN(0.2),
        LOWER(0.25),
        SNAP(.1);
        public final double servoPosition;
        IntakeWristPosition(double servoPosition) {
            this.servoPosition = servoPosition;
        }
    }

    //pivot servo positions
    public enum IntakeRotateAngle {
        ZERO(0.34), // Replace 0.0 with the actual servo position for 0 degrees
        FORTY_FIVE(0.145), // Replace 0.25 with the actual servo position for 45 degrees
        NINETY(0.0), // Replace 0.5 with the actual servo position for 90 degrees
        ONE_THIRTY_FIVE(0.52), // Replace 0.75 with the actual servo position for 135 degrees
        ONE_EIGHTY(0.675); // Replace 1.0 with the actual servo position for 180 degrees
        public final double servoPosition;
        IntakeRotateAngle(double servoPosition) {
            this.servoPosition = servoPosition;
        }
    }

    //elbow servo positions
    public enum IntakeElbowPosition {
        ELBOW_DOWN(.65),
        ELBOW_UP(0.0),
        ELBOW_SPECIMAN(0.2),
        ELBOW_INIT(0.0),
        SNAP_GAME_PIECE(0.80);
        public final double elbowPosition;
        IntakeElbowPosition(double servoPosition) {
            this.elbowPosition = servoPosition;
        }
    }

    /*
    lift servo positions (claw, wrist, elbow)
     */

    //claw servo positions
    public static final double
            LIFT_CLAW_POSITION_OPEN = 0.75,
            LIFT_CLAW_POSITION_CLOSED = 1.0;

    public enum LiftClawPosition {
        OPEN(0.75),
        CLOSED(1.0); // Replace 0.0 with the actual closed position
        public final double position;
        LiftClawPosition(double servoPosition) {
            this.position = servoPosition;
        }
    }
    //wrist servo positions
    public enum LiftWristPosition {
        TRANSFER(1.0),
        //SCORE(0.9),
        SCORE(0.3),

        SPECIMEN_RETRIEVAL(.4),
        ACQUIRE(0.4);

        public final double wristPosition;

        LiftWristPosition(double servoPosition) {
            this.wristPosition = servoPosition;
        }
    }

    //elbow servo positions
    public enum LiftElbowPosition {
        TRANSFER(0.1878),
        SCORE(0.65),
        SPECIMEN_RETRIEVAL(0.0);
        public final double position;
        LiftElbowPosition(double position) {
            this.position = position;
        }
    }
    //flags for servo states
    public boolean isIntakeClawClosed = true; // Initial state of the claw
    public boolean isRotateClawAtZero = true;
    public boolean isLiftElbowReadyforTransfer = true;
    private boolean isLiftClawClosed = true; // Initial state of the claw
    private boolean isLiftWristReadyforTransfer = true; // Initial state of the claw
    private boolean lastLeftBumperState = false;
    private boolean lastRightBumperState = false;

    final double INTAKE_COLLECT    = -.2;//1.0;
    final double INTAKE_OFF        =  0.0;
    final double INTAKE_DEPOSIT    =  0.75;//0.2;
    // Slide stopper Servo Positions (Lower/Raise)
    public static final double STOPPER_POSITION_RAISE = 0.0;
    public static final double STOPPER_POSITION_LOWER = 0.3;
    private Servo slideStopperServo;
    //endregion
    public Servo_Helper_Functions(HardwareMap hardwareMap) throws InterruptedException {
        initHardware(hardwareMap);
        setStartingPositions();
    }

    public void setStartingPositions() throws InterruptedException {
        initSlideStopperServo();
        moveLiftWristServotoTransferPosition();
        moveLiftElbowServotoTransferPosition();
        long liftMovementTimeMillis = 250; // EXAMPLE: 1 second, ADJUST THIS!
        sleep(liftMovementTimeMillis);
        closeIntakeClaw();
        rotateIntakeClawto0();
        moveIntakeArmToInitPosition();
        initIntakeElbow();
        openLiftClaw();
    }

    public void initHardware(@NonNull HardwareMap hardwareMap) {
        // Intake Servos
        intakeClawServo = hardwareMap.servo.get(INTAKE_CLAW_SERVO_NAME);
        intakeRotateServo = hardwareMap.servo.get(INTAKE_ROTATE_SERVO_NAME); // More descriptive name
        intakeWristServo = hardwareMap.servo.get(INTAKE_WRIST_SERVO_NAME);
        rightElbowServo = hardwareMap.servo.get(INTAKE_ELBOW_RIGHT_SERVO_NAME);
        leftElbowServo = hardwareMap.servo.get(INTAKE_ELBOW_LEFT_SERVO_NAME);
        leftElbowServo.setDirection(Servo.Direction.REVERSE);

        // Lift Servos
        liftClawServo = hardwareMap.servo.get(LIFT_CLAW_SERVO_NAME);
        liftElbowServo = hardwareMap.servo.get(LIFT_ELBOW_SERVO_NAME);
        liftElbowServoR = hardwareMap.servo.get(LIFT_ELBOWR_SERVO_NAME);
        liftElbowServoR.setDirection(Servo.Direction.REVERSE);
        liftWristServo = hardwareMap.servo.get(LIFT_WRIST_SERVO_NAME);

        //liftElbowServo.setDirection(Servo.Direction.REVERSE);
        slideStopperServo = hardwareMap.servo.get("slidestopper");
        intakeWristServo.setDirection(Servo.Direction.REVERSE);

    }
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(value, max));
    }
    public void openIntakeClaw() {
        setIntakeClawPosition(IntakeClawPosition.OPEN.position);
        isIntakeClawClosed = false;
    }
    public void closeIntakeClaw() {
        setIntakeClawPosition(IntakeClawPosition.CLOSED.position);
        isIntakeClawClosed = true;
    }
    public boolean isIntakeClawClosed() {
        return isIntakeClawClosed;
    }
    public void setIntakeClawPosition(double position) {
        position = clamp(position, 0.0, 1.0);
        intakeClawServo.setPosition(position);
    }
    public double getIntakeClawPosition() {
        return intakeClawServo.getPosition();
    }
    public void rotateIntakeClaw(double position) {
        position = clamp(position, 0.0, 1.0);
        intakeRotateServo.setPosition(position);
    }

    public void adjustRotateClaw(double increment) {
        double newPosition = getRotationIntakeClawPosition() + increment;
        rotateIntakeClaw(newPosition);
    }
    public void rotateIntakeClaw(@NonNull IntakeRotateAngle angle) {
        intakeRotateServo.setPosition(angle.servoPosition);
    }
    public double getRotationIntakeClawPosition() {
        return intakeRotateServo.getPosition();
    }
    public void rotateIntakeClawto0(){
        rotateIntakeClaw(IntakeRotateAngle.ZERO);
        isRotateClawAtZero = true;
    }
    public void rotateIntakeClawto45(){
        rotateIntakeClaw(IntakeRotateAngle.FORTY_FIVE);
        isRotateClawAtZero = false;
    }
    public void rotateIntakeClawto90(){
        rotateIntakeClaw(IntakeRotateAngle.NINETY);
        isRotateClawAtZero = false;
    }
    public void rotateIntakeClawto135(){
        rotateIntakeClaw(IntakeRotateAngle.ONE_THIRTY_FIVE);
        isRotateClawAtZero = false;
    }
    public void rotateIntakeClawto180(){
        rotateIntakeClaw(IntakeRotateAngle.ONE_EIGHTY);
        isRotateClawAtZero = false;
    }

    /****************************************************************************************************************************************/
    /* Intake wrist servo functions */
    public void raiseIntakeWrist() {
        moveIntakeWristPosition(IntakeWristPosition.RAISE);
    }

    public void initIntakeWrist() {
        moveIntakeWristPosition(IntakeWristPosition.INIT);
    }

    public void lowerIntakeWrist() {
        moveIntakeWristPosition(IntakeWristPosition.LOWER);//setIntakeWristPosition(INTAKE_WRIST_POSITION_LOWER);
    }
    public void incrementIntakeWrist(double increment) {
        setIntakeWristPosition(getIntakeWristPosition() + increment);
    }
    public double getIntakeWristPosition() {
        return intakeWristServo.getPosition();
    }
    public void moveIntakeWristPosition(@NonNull IntakeWristPosition position) {
        setIntakeWristPosition(position.servoPosition);
    }
    public double setIntakeWristPosition(double position) {
        position = clamp(position, 0.0, 1.0);
        intakeWristServo.setPosition(position);
        return position;
    }
    /* END Intake wrist servo functions */
    /****************************************************************************************************************************************/


    /****************************************************************************************************************************************/
    /* Intake elbow servo functions */
    public void lowerIntakeElbow() {
        moveIntakeElbowPosition(IntakeElbowPosition.ELBOW_DOWN);//setIntakeElbowPosition(INTAKE_ELBOW_POSITION_LOWER);
    }
    public void raiseIntakeElbow() {
        moveIntakeElbowPosition(IntakeElbowPosition.ELBOW_UP);//setIntakeElbowPosition(INTAKE_ELBOW_POSITION_RAISE);
    }

    public void initIntakeElbow() {
        moveIntakeElbowPosition(IntakeElbowPosition.ELBOW_INIT);//setIntakeElbowPosition(INTAKE_ELBOW_POSITION_RAISE);
    }

    public void adjustIntakeElbowPosition(double increment) {
        double newPosition = getLeftElbowPosition() + increment;
        setIntakeElbowPosition(newPosition);
    }
    public double getLeftElbowPosition() { return leftElbowServo.getPosition(); }
    public double getRightElbowPosition() { return rightElbowServo.getPosition(); }
    public void moveIntakeElbowPosition(@NonNull IntakeElbowPosition position) {
        setIntakeElbowPosition(position.elbowPosition);
    }
    public void setIntakeElbowPosition(double position) {
        double clampedPosition = clamp(position, 0.0, 1.0);
        rightElbowServo.setPosition(clampedPosition);
        leftElbowServo.setPosition(clampedPosition);
    }
    /****************************************************************************************************************************************/

    /****************************************************************************************************************************************/
    /* Lift claw servo functions */
    public void openLiftClaw() {
        liftClawServo.setPosition(LiftClawPosition.OPEN.position);//liftClawServo.setPosition(LIFT_CLAW_POSITION_OPEN);
        isLiftClawClosed = false;
    }
    public void closeLiftClaw() {
        liftClawServo.setPosition(LiftClawPosition.CLOSED.position);//liftClawServo.setPosition(LIFT_CLAW_POSITION_CLOSED);
        isLiftClawClosed = true;
    }
    public boolean isLiftClawClosed() {
        return isLiftClawClosed;
    }
    public void setLiftClawServoPosition(double position) {
        position = clamp(position, 0.0, 1.0);
        liftClawServo.setPosition(position);
    }
    public double getLiftClawServoPosition() {
        return liftClawServo.getPosition();
    }
    /****************************************************************************************************************************************/

    /****************************************************************************************************************************************/
    /* Lift wrist servo functions */
    private void moveLiftWristToPosition(@NonNull LiftWristPosition position) {
        liftWristServo.setPosition(position.wristPosition);
    }
    public double getLiftWristPosition (){
        return liftWristServo.getPosition();
    }

    public void moveLiftWristServotoAcquirePosition() {
        moveLiftWristToPosition(LiftWristPosition.ACQUIRE);
    }

    public void moveLiftWristServotoTransferPosition() {
        moveLiftWristToPosition(LiftWristPosition.TRANSFER);
        isLiftWristReadyforTransfer = true;
    }

    public void moveLiftWristServotoScorePosition() {
        moveLiftWristToPosition(LiftWristPosition.SCORE);
        isLiftWristReadyforTransfer = false;
    }
    public void moveLiftWristServotoGetSpecimen() {
        moveLiftWristToPosition(LiftWristPosition.SPECIMEN_RETRIEVAL);
        isLiftWristReadyforTransfer = false;
    }
    public void setLiftWristPosition(double position) {
        double clampedPosition = clamp(position, 0.0, 1.0);
        liftWristServo.setPosition(clampedPosition);
    }
    public boolean isLiftWristServoReadyforTransfer() {
        return isLiftWristReadyforTransfer;
    }
    /****************************************************************************************************************************************/


    /****************************************************************************************************************************************/
    /* lift elbow servo functions */
    public void adjustLiftElbowPosition(double increment) {
        double newPosition = getLiftElbowPosition() + increment;
        setLiftElbowPosition(newPosition);
    }
    public double getLiftElbowPosition() {
        return liftElbowServo.getPosition();
    }
    public void setLiftElbowPosition(double position) {
        double clampedPosition = clamp(position, 0.0, 1.0);
        liftElbowServo.setPosition(clampedPosition);
        liftElbowServoR.setPosition(clampedPosition);
    }
    private void moveLiftElbowToPosition(LiftElbowPosition position) {
        setLiftElbowPosition(position.position);
    }
    /****************************************************************************************************************************************/

    //game specific elbow move functions
    public void moveLiftElbowServotoTransferPosition() {
        moveLiftElbowToPosition(LiftElbowPosition.TRANSFER);
        isLiftElbowReadyforTransfer = true;
    }
    public void moveLiftElbowServotoScorePosition() {
        moveLiftElbowToPosition(LiftElbowPosition.SCORE);
        isLiftElbowReadyforTransfer = false;
    }
    public void moveLiftElbowServotoGetSpecimen() {
        moveLiftElbowToPosition(LiftElbowPosition.SPECIMEN_RETRIEVAL);
        isLiftElbowReadyforTransfer = false;
    }
    public boolean isLiftElbowServoReadyforTransfer() {
        return isLiftElbowReadyforTransfer;
    }



    /* combo functions *

     */

    public void moveLiftArmforGettingSpecimen() {
        moveLiftWristServotoGetSpecimen();
        moveLiftElbowServotoGetSpecimen();
        openLiftClaw();
    }

    public void moveIntakeArmforGettingSpecimen() {
        moveLiftArmforGettingSpecimen();
        //moveLiftElbowServotoGetSpecimen();
    }
    public void moveLiftArmforTransferSpecimen() {
        moveLiftWristServotoScorePosition();
        moveLiftElbowServotoScorePosition();
    }
    public void moveLiftArmforScoringSpecimen() {
        moveLiftWristServotoGetSpecimen();
        moveLiftElbowServotoGetSpecimen();
    }

    public void moveArmToAcquireGamePiece() {
        moveLiftWristServotoAcquirePosition();
        lowerIntakeElbow();
    }
    public void moveIntakeArmToTransferGamePiece() {
        raiseIntakeElbow();
        raiseIntakeWrist();

    }

    public void moveIntakeArmToInitPosition() {
        initIntakeElbow();
        initIntakeWrist();
    }

    public void snapGamePiece() {

        moveIntakeElbowPosition(IntakeElbowPosition.SNAP_GAME_PIECE);
        moveIntakeWristPosition(IntakeWristPosition.SNAP);
    }
    public void moveArmforScoringLift() {

    }
    public void moveLiftArmForTransfer() {
        moveLiftElbowServotoTransferPosition();
        moveLiftWristServotoTransferPosition();
        openLiftClaw();
    }
    public void toggleIntakeClawWithBumper(boolean leftBumper) {
        boolean leftBumperEdge = leftBumper && !lastLeftBumperState;

        if (leftBumperEdge) {
            // Toggle the claw state when the left bumper is pressed
            if (isIntakeClawClosed) {
                openIntakeClaw();
            } else {
                closeIntakeClaw();
            }
        }
        lastLeftBumperState = leftBumper;
    }

    public void toggleLiftClawWithBumper(boolean rightBumper) {
        boolean rightBumperEdge = rightBumper && !lastRightBumperState;

        if (rightBumperEdge) {
            // Toggle the claw state when the left bumper is pressed
            if (isLiftClawClosed) {
                openLiftClaw();
            } else {
                closeLiftClaw();
            }
        }
        lastRightBumperState = rightBumper;
    }

    public void togglePivotServoWithBumper(boolean rightBumper) {
        boolean rightBumperEdge = rightBumper && !lastRightBumperState;

        if (rightBumperEdge) {
            // Toggle the claw state when the left bumper is pressed
            if (isLiftWristReadyforTransfer) {
                moveLiftWristServotoScorePosition();
            } else {
                moveLiftWristServotoTransferPosition();
            }
        }
        lastRightBumperState = rightBumper;
    }

    public void intake_collect()  {


    }

    public void intake_off() {


    }
    public void intake_deposit(){


    }
    public void initSlideStopperServo() {
        slideStopperServo.setPosition(STOPPER_POSITION_LOWER);
    }
    public void raiseSlideStopper() {
        slideStopperServo.setPosition(STOPPER_POSITION_RAISE);
    }
    public void lowerSlideStopper() {
        slideStopperServo.setPosition(STOPPER_POSITION_LOWER);
    }
}
