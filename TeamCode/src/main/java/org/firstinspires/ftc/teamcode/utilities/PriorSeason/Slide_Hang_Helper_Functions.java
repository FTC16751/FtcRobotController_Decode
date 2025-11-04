package org.firstinspires.ftc.teamcode.utilities.PriorSeason;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Slide_Hang_Helper_Functions {
    private Telemetry telemetry;
    long timeoutMillis =5000;
    long startTime = System.currentTimeMillis();
    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;
    TouchSensor touchSensor;
    private final int LevelZero = 0;
    private final int LowPosition = 0;
    private final int MidPosition = 2500;
    private final int HighPosition = 4200;
    private final int MaxPosition = 4200;

    public Slide_Hang_Helper_Functions(HardwareMap hardwareMap, Telemetry telemetry ) {
        this.telemetry = telemetry;
        leftMotor = hardwareMap.get(DcMotorEx.class, "lefthang");
        rightMotor = hardwareMap.get(DcMotorEx.class, "righthang");

        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);


        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        currentState = LiftState.IDLE;


    }



    public void decreasePosition(int decreaseAmount) {
        int currentPosition;
        int newPosition;
        currentPosition = leftMotor.getCurrentPosition();
        newPosition = currentPosition - decreaseAmount;
        changePosition(newPosition);
    }
    public void increasePosition(int increaseAmount) {
        int currentPosition;
        int newPosition;
        currentPosition = leftMotor.getCurrentPosition();
        newPosition = currentPosition + increaseAmount;
        //if new position is over max position, set to max position
        if (newPosition > MaxPosition) {
            newPosition = MaxPosition;
        }
        changePosition(newPosition);
    }
    public void changePosition(int newPosition) {
        //if newposition is greater than max position set to max position
        if (newPosition > MaxPosition) {
            newPosition = MaxPosition;
        }
        leftMotor.setTargetPosition(newPosition);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setPower(1.0);
        rightMotor.setTargetPosition(newPosition);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setPower(1.0);
    }
    public void moveToPosition(int targetPosition) {
        double ERROR_THRESHOLD = 10.0;
        leftMotor.setTargetPositionTolerance(2);
        rightMotor.setTargetPositionTolerance(2);
        leftMotor.setTargetPosition(targetPosition);
        rightMotor.setTargetPosition(targetPosition);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setPower(1.0);
        rightMotor.setPower(1.0);
    }

    public void stopPower() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        //resetEncoder();
    }
    public void resetEncoder(){
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public int getLeftLiftPosition() {
        return leftMotor.getCurrentPosition();
    }
    public int getRightLiftPosition() {
        return rightMotor.getCurrentPosition();
    }
    public boolean slidesAtTargetPosition(int targetPosition) {
        // Implement logic to check if the arm is sufficiently close to the target position
        // Return true when the arm is at or close to the target
        return Math.abs(leftMotor.getCurrentPosition() - targetPosition) < 25
                && Math.abs(rightMotor.getCurrentPosition() - targetPosition) < 25;
    }
    public int getTargetPosition(LiftState currentState){
        if(currentState == LiftState.LEVEL_ZERO) {
            return LevelZero;
        }
        if(currentState == LiftState.LOW_POSITION) {
            return LowPosition;
        }
        if(currentState == LiftState.MID_POSITION) {
        return MidPosition;
        }
        if(currentState == LiftState.HIGH_POSITION) {
            return HighPosition;
        } else {
            return 0;
        }
    }

    public enum LiftState {
        LEVEL_ZERO,
        LOW_POSITION,
        MID_POSITION,
        HIGH_POSITION,
        IDLE
    }

    public long getStartTime() {
        return startTime;
    }

    public long getTimeoutMillis() {
        return timeoutMillis;
    }

    public long getelapsedtime(){
        return (System.currentTimeMillis() - startTime);
    }
    public void updateStateMachine(){
        runStateMachine();
    }
    private LiftState currentState;
    public void runStateMachine() {
        switch (currentState) {
            case LEVEL_ZERO:
                currentState = LiftState.LEVEL_ZERO;
                telemetry.addData("currentState:   ",currentState);
                moveToPosition(LevelZero);
                telemetry.addData("slidesAtTargetPosition:   ",slidesAtTargetPosition(LowPosition));
                if (slidesAtTargetPosition(LevelZero) == true) {
                    currentState = LiftState.IDLE;
                    updateStateMachine();
                }

                break;
            case LOW_POSITION:
                currentState = LiftState.LOW_POSITION;
                telemetry.addData("currentState:   ",currentState);
                moveToPosition(LowPosition);
                telemetry.addData("slidesAtTargetPosition:   ",slidesAtTargetPosition(LowPosition));
                if (slidesAtTargetPosition(LowPosition)) {
                    currentState = LiftState.IDLE;
                    updateStateMachine();
                }
                break;
            case MID_POSITION:
                currentState = LiftState.MID_POSITION;
                telemetry.addData("currentState:   ",currentState);
                moveToPosition(MidPosition);

                break;
            case HIGH_POSITION:
                currentState = LiftState.HIGH_POSITION;
                telemetry.addData("currentState:   ",currentState);
                moveToPosition(HighPosition);

                break;
            case IDLE:
                telemetry.addData("currentState:   ",currentState);
                stopPower();
                break;
            default:
                telemetry.addData("currentState:   ","Default");
                stopPower();
                break;
        }
    }
    public void setCurrentState(LiftState state) {
        currentState = state;
    }
    public LiftState getCurrentState() {
        return currentState;
    }
    public void sendTelemetryData() {
        telemetry.addData("Horizontal Left Lift Position: ", getLeftLiftPosition());
        telemetry.addData("Horizontal Right Lift Position: ", getRightLiftPosition());
    }
}
