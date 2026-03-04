package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Servo.Direction;

public class GobuildaRGB {
    private Servo rgbServo;

    public GobuildaRGB(HardwareMap hwMap) {
        rgbServo = hwMap.get(Servo.class, "rgb");
        rgbServo.setPosition(0.15);
        rgbServo.setDirection(Direction.FORWARD);
    }

    public void setRed() { rgbServo.setPosition(0.15); }
    public void setGreen() { rgbServo.setPosition(0.50); }
    public void setBlue() { rgbServo.setPosition(0.85); }
    public void setYellow() { rgbServo.setPosition(0.25); }
    public void setAllianceColor(boolean isRed) {
        rgbServo.setPosition(isRed ? 0.15 : 0.85);
    }
    public void setOff() { rgbServo.setPosition(0.0); }

    public void periodic() {
        // if needed for updates
    }
}
