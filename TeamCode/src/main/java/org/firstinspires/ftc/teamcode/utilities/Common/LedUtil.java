package org.firstinspires.ftc.teamcode.utilities.Common;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * A utility class for controlling the goBILDA RGB Indicator Light (3118-0808-0002).
 * This class encapsulates the servo control and provides simple, human-readable
 * methods to set the robot's status color.
 */
public class LedUtil {

    private final Servo ledServo;

    // These constants are taken directly from the goBILDA Product Insight #4 datasheet.
    // They represent the servo position values for specific colors.
    public static final class Color {
        public static final double OFF    = 0.0;
        public static final double RED    = 0.277;
        public static final double ORANGE = 0.333;
        public static final double YELLOW = 0.388;
        public static final double GREEN  = 0.500;
        public static final double BLUE   = 0.611;
        public static final double VIOLET = 0.720;
        public static final double WHITE  = 1.0;
    }


    public LedUtil(HardwareMap hardwareMap, String deviceName) {
        try {
            ledServo = hardwareMap.get(Servo.class, deviceName);
            // Set a default color on initialization.
            setColor(Color.OFF);
        } catch (Exception e) {
            throw new RuntimeException("Could not find or initialize LED servo: " + deviceName, e);
        }
    }


    public void setColor(double colorValue) {
        if (colorValue >= 0.0 && colorValue <= 1.0) {
            ledServo.setPosition(colorValue);
        }
    }

    public double getColor(){
        return ledServo.getPosition();
    }
}