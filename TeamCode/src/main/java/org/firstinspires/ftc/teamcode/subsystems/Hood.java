package org.firstinspires.ftc.teamcode.subsystems;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class Hood {
    private final Servo right;

    public static double LOW_RIGHT = 1.0;

    public static double HIGH_RIGHT = 0.05;

    public static double upIncrement = .05;
    public static double downIncrement = .05;

    public Hood(HardwareMap hw) {
        right = hw.get(Servo.class, "hoodServoRight");
    }

    private double clamp(double v) {
        return Math.max(0.0, Math.min(1.0, v));
    }

    public void set(double rightPos) {
        right.setPosition(clamp(rightPos));
    }

    public double getRightPosition() {
        return right.getPosition();
    }

    public void low() {
        set(LOW_RIGHT);
    }
    public void high() {
        set(HIGH_RIGHT);
    }

    public void moveUp() {
        set(getRightPosition() - upIncrement);
    }
    public void moveDown() { set(getRightPosition() + downIncrement); }
}
