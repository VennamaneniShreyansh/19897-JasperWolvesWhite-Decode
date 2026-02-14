package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
public class Turret {
    public static double error = 0, power = 0, manualPower = 0;
    public static int offsetTicks = 0;
    public static double rpt = 0.00653;

    public final DcMotorEx m;
//    private PIDFController primaryPID, secondaryPID;
    public static double target = 0;
//    public static double pidfSwitch = 5;
//    public static double kp = 0.004, kf = 0.0001, kd = 0.000, sp = .03, sf = 0, sd = 0.0001;
//

    // New Stuff
    private double integralSum = 0;
    private double lastError = 0;
    private double lastReference = 0;
    private double previousFilterEstimate = 0;
    private double currentFilterEstimate = 0;

    private final double a = 0.8; // low-pass filter constant
    private final double maxIntegralSum = 1000; // anti-windup limit

    // PID constants; use new tuning values from Homeostasis model
    public static double Kp = 0.0076;
    public static double Ki = 0.0;
    public static double Kd = 0.0002;

    private final ElapsedTime timer = new ElapsedTime();


    public static boolean on = true, manual = false;



    public Turret(HardwareMap hardwareMap) {
        m = hardwareMap.get(DcMotorEx.class, "turret");
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        primaryPID = new PIDFController(new PIDFCoefficients(kp, 0, kd, kf));
//        secondaryPID = new PIDFController(new PIDFCoefficients(sp, 0, sd, sf));
    }

    public void setTurretTarget(double ticks) {
        target = Math.max(-350, Math.min(345, ticks));
    }


    public double getTurretTarget() { return target; }
    public double getTurret() {
        return m.getCurrentPosition() + offsetTicks;
    }

public void periodic() {
    if (!on) {
        m.setPower(0);
        return;
    }
    if (manual) {
        m.setPower(manualPower);
        return;
    }
    double currentPosition = getTurret();
    double reference = getTurretTarget();

    double error = reference - currentPosition;

    double elapsedTime = timer.seconds();
    if (elapsedTime <= 0) elapsedTime = 1e-3;
    double errorChange = error - lastError;

    currentFilterEstimate = (a * previousFilterEstimate) + ((1 - a) * errorChange);
    previousFilterEstimate = currentFilterEstimate;

    double derivative = currentFilterEstimate / elapsedTime;

    integralSum += error * elapsedTime;
    integralSum = Math.max(-maxIntegralSum, Math.min(maxIntegralSum, integralSum));

    if (reference != lastReference) {
        integralSum = 0;
    }

    double output = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

    m.setPower(Math.max(-1, Math.min(1, output)));

    lastError = error;
    lastReference = reference;
    timer.reset();
}



    public void adjustOffsetRight(int ticks) {
        offsetTicks -= ticks;
        offsetTicks = Math.max(-50, Math.min(50, offsetTicks));
    }
    public void adjustOffsetLeft(int ticks) {
        offsetTicks += ticks;
        offsetTicks = Math.max(-50, Math.min(50, offsetTicks));
    }
    public void resetOffset() {
        offsetTicks = 0;
    }
    public double getOffsetTicks() { return offsetTicks; }

    public void automatic() {
        manual = false;
    }
    public void on() { on = true; }
    public void off() { on = false; }

    public double getYaw() { return normalizeAngle(getTurret() * rpt); }

    public void setYaw(double radians) {
        radians = normalizeAngle(radians);
        setTurretTarget(radians / rpt);
    }

    public void face(Pose targetPose, Pose robotPose) {
        double dx = targetPose.getX() - robotPose.getX();
        double dy = targetPose.getY() - robotPose.getY();

        double targetAngle = Math.atan2(dy, dx);
        double turretYaw = normalizeAngle(targetAngle -
                robotPose.getHeading());

        setYaw(turretYaw);
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

    public void incrementTurretRight() {
        setTurretTarget(target -= 20);
    }
    public void incrementTurretLeft() {
        setTurretTarget(target += 20);
    }

    public double angleDegreesToTicks(double angleDegrees) {
        double normalizedDegrees = normalizeAngleDegrees(angleDegrees);
        normalizedDegrees = Math.max(-270.0, Math.min(270.0, normalizedDegrees));
        double radians = Math.toRadians(normalizedDegrees);
        return radians / rpt;
    }
    private double normalizeAngleDegrees(double angleDegrees) {
        double angle = angleDegrees % 360.0;
        if (angle > 180.0) angle -= 360.0;
        if (angle <= -180.0) angle += 360.0;
        return angle;
    }



    public double getError() { return error; }
}
