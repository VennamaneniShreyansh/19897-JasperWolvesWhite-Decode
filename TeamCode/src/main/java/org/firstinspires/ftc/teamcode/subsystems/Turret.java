package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Turret {
    public static double error = 0, power = 0, manualPower = 0;

    // output / input = 100 / 40 = 2.5 = Gear ratio
    // ticksPerTurretRev = motorTicksPerRev × gearRatio
    // ticksPerTurretRev = 384.5 × 2.5
    // ticksPerTurretRev = 961.25 ticks, so this = 360 degrees turret rotation
    // rpt = (2π) / ticksPerTurretRev
    // rpt = 6.283185307 / 961.25
    // rpt ~ 0.00653 radians per tick
    public static double rpt = 0.00653;

    public final DcMotorEx m;
    private PIDFController primaryPID, secondaryPID; // pidf controller for turret
    public static double target = 0;
    public static double pidfSwitch = 5; // target for turret
    // p is primary and s is secondary
    public static double kp = 0.004, kf = 0.0001, kd = 0.000, sp = .0055, sf = 0, sd = 0.0001;

    public static boolean on = true, manual = false;

    public Turret(HardwareMap hardwareMap) {
        m = hardwareMap.get(DcMotorEx.class, "turret");
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        primaryPID = new PIDFController(new PIDFCoefficients(kp, 0, kd, kf));
        secondaryPID = new PIDFController(new PIDFCoefficients(sp, 0, sd, sf));
    }

    private void setTurretTarget(double ticks) {
//        target = Math.max(-240, Math.min(268, ticks));
        target = Math.max(-240, Math.min(300, ticks));;
    }

    public double getTurretTarget() {
        return target;
    }

    public double getTurret() {
        return m.getCurrentPosition();
    }

    public void periodic() {
        if (on) {
            if (manual) {
                m.setPower(manualPower);
                return;
            }
            // allows for tuning while robot runs
            primaryPID.setCoefficients(new PIDFCoefficients(kp, 0, kd, kf));
            secondaryPID.setCoefficients(new PIDFCoefficients(sp, 0, sd, sf));
            // difference between the current position and the target positon
            error = getTurretTarget() - getTurret();
            // Decides which PID to use secondary or primary based on pidfSwitch
            if (Math.abs(error) > pidfSwitch) {
                primaryPID.updateError(error);
                primaryPID.updateFeedForwardInput(Math.signum(error));
                power = primaryPID.run();
            } else {
                secondaryPID.updateError(error);
                power = secondaryPID.run();
            }

            m.setPower(power);
        } else {
            m.setPower(0);
        }
    }

    public void manual(double power) {
        manual = true;
        manualPower = power;
    }

    public void automatic() {
        manual = false;
    }

    public void on() {
        on = true;
    }

    public void off() {
        on = false;
    }

    public double getYaw() {
        return normalizeAngle(getTurret() * rpt);
    }



    public void setYaw(double radians) {
        radians = normalizeAngle(radians);
        setTurretTarget(radians/rpt);
    }

    public void addYaw(double radians) {
        setYaw(getYaw() + radians);
    }

    public void face(Pose targetPose, Pose robotPose) {
        double angleToTargetFromCenter = Math.atan2(targetPose.getY() - robotPose.getY(), targetPose.getX() - robotPose.getX());
        double robotAngleDiff = normalizeAngle(angleToTargetFromCenter - robotPose.getHeading());
        setYaw(robotAngleDiff);
    }

    public void resetTurret() {
        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setTurretTarget(0);
    }


    public static double normalizeAngle(double angleRadians) {
        double angle = angleRadians % (Math.PI * 2D);
        if (angle <= -Math.PI) angle += Math.PI * 2D;
        if (angle > Math.PI) angle -= Math.PI * 2D;
        return angle;
    }

    public double getError() {
        return error;
    }

    public boolean isReady() {
        return Math.abs(getError()) < 30;
    }
}