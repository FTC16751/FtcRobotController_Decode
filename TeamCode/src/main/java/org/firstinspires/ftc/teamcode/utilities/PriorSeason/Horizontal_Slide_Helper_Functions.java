package org.firstinspires.ftc.teamcode.utilities.PriorSeason;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Horizontal_Slide_Helper_Functions {
    private Telemetry telemetry;
    long timeoutMillis =5000;
    long startTime = System.currentTimeMillis();

    TouchSensor touchSensor;
    private final double slidesRetractPosition = 0.30;
    private final double slideExtendPosition = 0.02;
    private final double slideMidPosition = 0.22;
    private final double slideHighPosition = 0.20;
    private Servo leftSlideServo, rightSlideServo;
    public Horizontal_Slide_Helper_Functions(HardwareMap hardwareMap, Telemetry telemetry ) {
        this.telemetry = telemetry;

        currentState = SlideState.IDLE;
        touchSensor = hardwareMap.get(TouchSensor.class, "lifttouchsensor");
        leftSlideServo = hardwareMap.get(Servo.class, "leftslide");
        leftSlideServo.setDirection(Servo.Direction.FORWARD); //https://javadoc.io/doc/org.firstinspires.ftc/RobotCore/latest/com/qualcomm/robotcore/hardware/Servo.Direction.html

        rightSlideServo = hardwareMap.get(Servo.class, "rightslide");
        rightSlideServo.setDirection(Servo.Direction.REVERSE); //https://javadoc.io/doc/org.firstinspires.ftc/RobotCore/latest/com/qualcomm/robotcore/hardware/Servo.Direction.html
        initStartPosition();
    }

    public Boolean isTouchSensorPressed() {
        return touchSensor.isPressed();
    }

    public void decreasePosition(double decreaseAmount) {
        changePosition(getLeftHorizontalSlidePosition() - decreaseAmount);
    }
    public void increasePosition(double increaseAmount) {
        changePosition(getLeftHorizontalSlidePosition() + increaseAmount);
    }
    public void changePosition(double position) {
        position = clamp(position, 0.0, 1.0);
        leftSlideServo.setPosition(position);
        rightSlideServo.setPosition(position);
    }
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(value, max));
    }
    public double getLeftHorizontalSlidePosition() {
        return leftSlideServo.getPosition();
    }
    public double getRightHorizontalSlidePosition() {
        return rightSlideServo.getPosition();
    }


    public void moveToPosition(double targetPosition) {
        changePosition(targetPosition);
    }

    public void stopSlides() {
    }
    public void resetEncoder(){

    }
    public double getLeftSlidePosition() {
        return leftSlideServo.getPosition();
    }
    public double getRightSlidePosition() {
        return rightSlideServo.getPosition();
    }
    public boolean slidesAtTargetPosition(double targetPosition) {
        return true;//until we get sensors to measure
    }
    public double getTargetPosition(SlideState currentState){
        if(currentState == SlideState.LEVEL_ZERO) {
            return slidesRetractPosition;
        }
        if(currentState == SlideState.LOW_POSITION) {
            return slideExtendPosition;
        }
         else {
            return slidesRetractPosition;
        }
    }

    public enum SlideState {
        LEVEL_ZERO,
        LOW_POSITION,
        MID_POSITION,
        HIGH_POSITION,
        IDLE
    }


    public void initStartPosition() {
        moveToPosition(slidesRetractPosition);
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
    private SlideState currentState;
    public void runStateMachine() {
        switch (currentState) {
            case LEVEL_ZERO:
                currentState = SlideState.LEVEL_ZERO;
                telemetry.addData("currentState:   ",currentState);
                moveToPosition(slidesRetractPosition);
                telemetry.addData("slidesAtTargetPosition:   ",slidesAtTargetPosition(slideExtendPosition));
                if (slidesAtTargetPosition(slidesRetractPosition)) {
                    currentState = SlideState.IDLE;
                    updateStateMachine();
                }

                break;
            case LOW_POSITION:
                currentState = SlideState.LOW_POSITION;
                telemetry.addData("currentState:   ",currentState);
                moveToPosition(slideExtendPosition);
                telemetry.addData("slidesAtTargetPosition:   ",slidesAtTargetPosition(slideExtendPosition));
                if (slidesAtTargetPosition(slideExtendPosition)) {
                    currentState = SlideState.IDLE;
                    updateStateMachine();
                }
                break;
            case MID_POSITION:
                currentState = SlideState.MID_POSITION;
                telemetry.addData("currentState:   ",currentState);
                moveToPosition(slideMidPosition);
                telemetry.addData("slidesAtTargetPosition:   ",slidesAtTargetPosition(slideMidPosition));
                if (slidesAtTargetPosition(slideMidPosition)) {
                    currentState = SlideState.IDLE;
                    updateStateMachine();
                }
                break;
            case HIGH_POSITION:
                currentState = SlideState.HIGH_POSITION;
                telemetry.addData("currentState:   ",currentState);
                moveToPosition(slideHighPosition);

                break;
            case IDLE:
                telemetry.addData("currentState:   ",currentState);
                stopSlides();
                break;
            default:
                telemetry.addData("currentState:   ","Default");
                stopSlides();
                break;
        }
    }
    public void setCurrentState(SlideState state) {
        currentState = state;
    }
    public SlideState getCurrentState() {
        return currentState;
    }
    public void sendTelemetryData() {
        telemetry.addData("Horizontal Lift currentState:   ",currentState);
        telemetry.addData("Horizontal Left Lift Position: ", getLeftSlidePosition());
        telemetry.addData("Horizontal Right Lift Position: ", getRightSlidePosition());
        telemetry.addData("Horizontal touch sensor: ", isTouchSensorPressed());

    }
}
