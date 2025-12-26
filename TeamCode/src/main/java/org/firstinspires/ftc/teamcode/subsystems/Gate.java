package org.firstinspires.ftc.teamcode.subsystems;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class Gate {
    private final Servo servo;

    public static double open = 0.61;
    public static double close = 0.0;

    public Gate(HardwareMap hw) {
        servo = hw.get(Servo.class, "servoGate");
    }

    public void openGate() {
        servo.setPosition(open);
    }

    public void closeGate() {
        servo.setPosition(close);
    }

    public void toggle() {
        if (Math.abs(servo.getPosition()-open) < .04)
            closeGate();
        else
            openGate();
    }
}

