package org.firstinspires.ftc.teamcode.subsystems;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class Hood {

    private final Servo left;
    private final Servo right;

    // .6494 r
    // .35 l

    // .95
    // .05

    public static double LOW_LEFT = 0.0;
    public static double LOW_RIGHT = 1.0;

    public static double HIGH_LEFT = 1.0;
    public static double HIGH_RIGHT = 0.0;

    public static double upIncrement = .05;
    public static double downIncrement = .05;

    public Hood(HardwareMap hw) {
        left  = hw.get(Servo.class, "hoodServoLeft");
        right = hw.get(Servo.class, "hoodServoRight");
        low();
    }

    private double clamp(double v) {
        return Math.max(0.0, Math.min(1.0, v));
    }

    public void set(double leftPos, double rightPos) {
        left.setPosition(clamp(leftPos));
        right.setPosition(clamp(rightPos));
    }

    public void setPos(double pos) {
        left.setPosition(clamp(pos));
        right.setPosition(clamp(1-pos));
    }

    public double getLeftPosition() {
        return left.getPosition();
    }
    public double getRightPosition() {
        return right.getPosition();
    }

    public void low() {
        set(LOW_LEFT, LOW_RIGHT);
    }
    public void high() {
        set(HIGH_LEFT, HIGH_RIGHT);
    }

    public void moveUp() {
        set(getLeftPosition() + upIncrement, getRightPosition() - upIncrement);
    }
    public void moveDown() {
        set(getLeftPosition() - downIncrement, getRightPosition() + downIncrement);
    }
}
