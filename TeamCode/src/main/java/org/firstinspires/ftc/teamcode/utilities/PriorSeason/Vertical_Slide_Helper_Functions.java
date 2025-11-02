package org.firstinspires.ftc.teamcode.utilities.PriorSeason;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;
import java.util.List;

public class Vertical_Slide_Helper_Functions {
    private Telemetry telemetry;
    private DcMotorEx vSlide,vSlide2,vSlide3,vSlide4;
    TouchSensor touchSensor;
    private final int liftLevelZero = 0;
    private final int liftHighBasketPosition = 2200;
    private final int liftHighSubmersiblePosition = 650;//NOT USED
    public Vertical_Slide_Helper_Functions(HardwareMap hardwareMap,Telemetry telemetry) {
        //vlift and vlift2 are right lift motors (both are forward direction)
        vSlide = hardwareMap.get(DcMotorEx.class, "vlift");
        vSlide2 = hardwareMap.get(DcMotorEx.class, "vlift2");

        //vlift 3 and vlift4 are left lift motors (both are reversed
        vSlide3 = hardwareMap.get(DcMotorEx.class, "vlift3");
        vSlide4 = hardwareMap.get(DcMotorEx.class, "vlift4");

        //vSlide.setDirection(DcMotor.Direction.REVERSE);
        vSlide.setDirection(DcMotor.Direction.FORWARD);
        vSlide2.setDirection(DcMotor.Direction.FORWARD);

        vSlide3.setDirection(DcMotor.Direction.REVERSE);
        vSlide4.setDirection(DcMotor.Direction.REVERSE);


        vSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        vSlide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vSlide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vSlide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        vSlide3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vSlide3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vSlide3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        vSlide4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vSlide4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vSlide4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        currentState = SlideState.IDLE;
        touchSensor = hardwareMap.get(TouchSensor.class, "vlifttouchsensor");
        this.telemetry = telemetry;

    }

    public Boolean isTouchSensorPressed() {
        return touchSensor.isPressed();
    }

    public void decreasePosition(int decreaseAmount) {
        int currentPosition;
        int newPosition;
        currentPosition = vSlide.getCurrentPosition();
        newPosition = currentPosition - decreaseAmount;
        changePosition(newPosition);
    }
    public void increasePosition(int increaseAmount) {
        int currentPosition;
        int newPosition;
        currentPosition = vSlide.getCurrentPosition();
        newPosition = currentPosition + increaseAmount;
        changePosition(newPosition);
    }
    public void changePosition(int newPosition) {
        vSlide.setTargetPosition(newPosition);
        vSlide2.setTargetPosition(newPosition);
        vSlide3.setTargetPosition(newPosition);
        vSlide4.setTargetPosition(newPosition);

        vSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vSlide3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vSlide4.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        vSlide.setPower(1.0);
        vSlide2.setPower(1.0);
        vSlide3.setPower(1.0);
        vSlide4.setPower(1.0);
    }

    public void moveToPosition(int targetPosition) {
        double ERROR_THRESHOLD = 10.0;
        vSlide.setTargetPositionTolerance(15);
        vSlide.setTargetPosition(targetPosition);
        vSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vSlide.setPower(1.0);

        vSlide2.setTargetPositionTolerance(15);
        vSlide2.setTargetPosition(targetPosition);
        vSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vSlide2.setPower(1.0);

        vSlide3.setTargetPositionTolerance(15);
        vSlide3.setTargetPosition(targetPosition);
        vSlide3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vSlide3.setPower(1.0);

        vSlide4.setTargetPositionTolerance(15);
        vSlide4.setTargetPosition(targetPosition);
        vSlide4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vSlide4.setPower(1.0);
    }
    public void stopSlides() {
        vSlide.setPower(0);
        vSlide2.setPower(0);
        vSlide3.setPower(0);
        vSlide4.setPower(0);
    }
    public void resetEncoder(){
        List<DcMotor> motors = Arrays.asList(vSlide, vSlide2, vSlide3, vSlide4);

        for (DcMotor motor : motors) {
            if (motor != null) { // Good practice to check for null
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }
    public int getLeftLiftPosition() {
        return vSlide.getCurrentPosition();
    }
    public boolean slidesAtTargetPosition(int targetPosition) {
        // Implement logic to check if the arm is sufficiently close to the target position
        // Return true when the arm is at or close to the target
        return Math.abs(vSlide.getCurrentPosition() - targetPosition) < 10;
    }
    public int getTargetPosition(SlideState currentState){
        if(currentState == SlideState.LEVEL_ZERO) {
            return liftLevelZero;
        }
        if(currentState == SlideState.LOW_POSITION) {
            return liftHighBasketPosition;
        }
         if(currentState == SlideState.HIGH_POSITION) {
            return liftHighSubmersiblePosition;
        } else {
            return 0;
        }
    }
    public enum SlideState {
        LEVEL_ZERO,
        LOW_POSITION,
        HIGH_POSITION,
        IDLE
    }
    public void updateStateMachine(){
        runStateMachine();
    }
    private SlideState currentState;
    public void setCurrentState(SlideState state) {
        currentState = state;
    }
    public SlideState getCurrentState() {
        return currentState;
    }
    public void runStateMachine() {
        switch (currentState) {
            case LEVEL_ZERO:
                currentState = SlideState.LEVEL_ZERO;
                telemetry.addData("currentState:   ",currentState);
                moveToPosition(liftLevelZero);
                break;
            case LOW_POSITION:
                currentState = SlideState.LOW_POSITION;
                telemetry.addData("currentState:   ",currentState);
                moveToPosition(liftHighBasketPosition);
                break;
            case HIGH_POSITION:
                currentState = SlideState.HIGH_POSITION;
                telemetry.addData("currentState:   ",currentState);
                moveToPosition(liftHighSubmersiblePosition);
                break;
            case IDLE:
                break;
            default:
                stopSlides();
                break;
        }
    }
public void sendTelemetryData() {
    telemetry.addData("Vertical Lift currentState:   ",currentState);
    telemetry.addData("Vertical Lift1 Position: ", vSlide.getCurrentPosition());
    telemetry.addData("Vertical Lift2 Position: ", vSlide2.getCurrentPosition());
    telemetry.addData("Vertical Lift3 Position: ", vSlide3.getCurrentPosition());
    telemetry.addData("Vertical Lift4 Position: ", vSlide4.getCurrentPosition());
    telemetry.addData("Vertical Slide getTargetPosition ", getTargetPosition(currentState));
    telemetry.addData("Vertical Slide slidesAtTargetPosition ",slidesAtTargetPosition(getTargetPosition(currentState)));
    telemetry.addData("Vertical touch sensor: ", isTouchSensorPressed());
}
}
