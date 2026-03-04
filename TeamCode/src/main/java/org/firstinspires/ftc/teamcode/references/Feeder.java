package org.firstinspires.ftc.teamcode.references;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class Feeder {
    private final Servo servo;

    public static double in = 0.0;
    public static double out = 0.6;

    public Feeder(HardwareMap hw) {
        servo = hw.get(Servo.class, "feeder");
    }
    public void push() {
        servo.setPosition(out);
    }

    public void retract() {
        servo.setPosition(in);
    }

    public void toggle() {
        if (servo.getPosition() == in)
            push();
        else
            retract();
    }
}

