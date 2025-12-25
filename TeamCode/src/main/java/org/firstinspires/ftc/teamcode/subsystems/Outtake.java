package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Outtake {

    private final DcMotorEx left, right;

    public static double HIGH_RPM = 4500;
    public static double LOW_RPM  = 3000;
    public static double TICKS_PER_REV = 28;

    public static double kP = 20, kI = 0, kD = 1.0;
    public static double kF = 32767.0 / (6000 * TICKS_PER_REV);

    public double targetRPM = 0;
    public boolean enabled = false;

    public Outtake(HardwareMap hw) {
        left  = hw.get(DcMotorEx.class, "outtakeLeft");
        right = hw.get(DcMotorEx.class, "outtakeRight");

        left.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);



        left.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        right.setVelocityPIDFCoefficients(kP, kI, kD, kF);
    }

    public void shootHigh() {
        targetRPM = HIGH_RPM;
        enabled = true;
    }

    public void shootLow() {
        targetRPM = LOW_RPM;
        enabled = true;
    }

    public void stop() {
        enabled = false;
        left.setVelocity(0);
        right.setVelocity(0);
    }

    public void periodic() {
        if (!enabled) return;

        double ticks = targetRPM * TICKS_PER_REV / 60.0;
        left.setVelocity(-ticks);
        right.setVelocity(ticks);
    }

    public boolean atTarget() {
        return Math.abs(getRPM(left) - targetRPM) < 75 &&
                Math.abs(getRPM(right) - targetRPM) < 75;
    }

    private double getRPM(DcMotorEx m) {
        return m.getVelocity() * 60 / TICKS_PER_REV;
    }
}
