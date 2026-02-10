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
    public static double Kp = 0.0067;
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
    public double getTurret() { return m.getCurrentPosition(); }

//    public void periodic() {
//        if (!on) { m.setPower(0); return; }
//
//        if (manual) { m.setPower(manualPower); return; }
//
//        primaryPID.setCoefficients(new PIDFCoefficients(kp, 0, kd, kf));
//        secondaryPID.setCoefficients(new PIDFCoefficients(sp, 0, sd, sf));
//
//        error = getTurretTarget() - getTurret();
//        if (Math.abs(error) > pidfSwitch) {
//            primaryPID.updateError(error);
//            primaryPID.updateFeedForwardInput(Math.signum(error));
//            power = primaryPID.run();
//        } else {
//            secondaryPID.updateError(error);
//            power = secondaryPID.run();
//        }
//
//        m.setPower(power);
//    }
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



    public void manual(double power) {
        manual = true;

        if (Math.abs(power) < 0.05) {
            manualPower = 0;
            m.setPower(0);
        } else {
            manualPower = power;
        }
    }

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

    public double getError() { return error; }
}
