package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {

    private final DcMotor fl, fr, bl, br;

    // Tunables
    public static double DRIVE_SPEED = 0.95;
    public static double STRAFE_MULTIPLIER = 1.1;
    public static double ROTATION_MULTIPLIER = 0.6;
    public static boolean IS_BRAKING = true;

    public Drivetrain(HardwareMap hw) {
        fl = hw.get(DcMotor.class, "fl");
        fr = hw.get(DcMotor.class, "fr");
        bl = hw.get(DcMotor.class, "bl");
        br = hw.get(DcMotor.class, "br");

        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);

        setBrake(IS_BRAKING);
    }

    public void drive(double y, double x, double rx) {
        x *= STRAFE_MULTIPLIER;
        rx *= ROTATION_MULTIPLIER;

        double flPower = y + x + rx;
        double frPower = y - x - rx;
        double blPower = y - x + rx;
        double brPower = y + x - rx;

        fl.setPower(clamp(flPower) * DRIVE_SPEED);
        fr.setPower(clamp(frPower) * DRIVE_SPEED);
        bl.setPower(clamp(blPower) * DRIVE_SPEED);
        br.setPower(clamp(brPower) * DRIVE_SPEED);
    }

    public void stop() {
        setPowers(0, 0, 0, 0);
    }

    private void setPowers(double flP, double frP, double blP, double brP) {
        fl.setPower(flP);
        fr.setPower(frP);
        bl.setPower(blP);
        br.setPower(brP);
    }

    private void setBrake(boolean brake) {
        DcMotor.ZeroPowerBehavior mode =
                brake ? DcMotor.ZeroPowerBehavior.BRAKE
                        : DcMotor.ZeroPowerBehavior.FLOAT;

        fl.setZeroPowerBehavior(mode);
        fr.setZeroPowerBehavior(mode);
        bl.setZeroPowerBehavior(mode);
        br.setZeroPowerBehavior(mode);
    }

    private double clamp(double v) {
        return Math.max(-1.0, Math.min(1.0, v));
    }
}
