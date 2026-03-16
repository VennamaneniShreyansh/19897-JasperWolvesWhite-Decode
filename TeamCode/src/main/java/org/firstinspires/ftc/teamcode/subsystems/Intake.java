package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Intake {
    private final DcMotorEx i;

    public static double OFF = 0;
    public static double IN = -1;
    public static double OUT = 1;
    public static double SLOW_IN = -1;

    public Intake(HardwareMap hardwareMap) {
        i = hardwareMap.get(DcMotorEx.class, "intake");
        set(OFF);
    }

    public void set(double power) {
        i.setPower(power);
    }
    public void spinIn()  {
        set(IN);
    }
    public void slowSpinIn() {
        set(SLOW_IN);
    }
    public void spinOut() {
        set(OUT);
    }
    public void spinOff() {
        set(OFF);
    }
}