//package org.firstinspires.ftc.teamcode.subsystems;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.pedropathing.geometry.Pose;
//import com.qualcomm.robotcore.hardware.AnalogInput;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.acmerobotics.dashboard.config.Config;
//import com.bylazar.configurables.annotations.Configurable;
////
//@Config
//@Configurable
//public class Turret {
//    public static double error = 0, power = 0, manualPower = 0;
//    private final AnalogInput encoder;
//    public final DcMotorEx m;
//    public static double targetDegrees = 0;
//
//    private double integralSum = 0;
//    private double lastError = 0;
//    private double lastReference = 0;
//    private double previousFilterEstimate = 0;
//    private double currentFilterEstimate = 0;
//
//    private final double a = 0.8; // low-pass filter constant
//    private final double maxIntegralSum = 45; // anti-windup limit (in degrees)
//
//    public static double Kp = 0.0143;
//    public static double Ki = 0.0;
//    public static double Kd = 0.0001;
//
//
//    private final ElapsedTime timer = new ElapsedTime();
//    public static boolean on = true, manual = false;
//
//    public Turret(HardwareMap hardwareMap) {
//        encoder = hardwareMap.get(AnalogInput.class, "turretEncoder");
//        m = hardwareMap.get(DcMotorEx.class, "turret");
//        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
////        m.setDirection(DcMotor.Direction.REVERSE);
//        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//    }
//
//
//
//    public static double ENCODER_ZERO_OFFSET_VOLTAGE = -3.192;
//    public static double lastAngle = 0;
//    public static double unwrappedAngle = 0;
//
//    public double getFinalVoltage() {
//        return encoder.getVoltage();
//    }
//
//    public double getTurretDegrees() {
//        double voltage = encoder.getVoltage() + ENCODER_ZERO_OFFSET_VOLTAGE;
//        double angle = (voltage / 3.3) * -360.0;
//        return angle;
//    }
//
//
//
//    public double getVoltage() {
//        return encoder.getVoltage();
//    }
//    public double getTurretTarget() { return targetDegrees; }
////    public void setTurretTarget(double degrees) {
////        targetDegrees = Math.max(-170, Math.min(135, degrees));
////    }
//
//    public static final double MIN_LIMIT_DEGREES = -175;
//    public static final double MAX_LIMIT_DEGREES = 145;
//
//    public void setTurretTarget(double degrees) {
//        targetDegrees = Math.max(MIN_LIMIT_DEGREES, Math.min(MAX_LIMIT_DEGREES, degrees));
//    }
//
//        public void periodic() {
//            if (!on) {
//                m.setPower(0);
//                return;
//            }
//            if (manual) {
//                m.setPower(Math.max(-0.3, Math.min(0.3, manualPower)));
//                return;
//            }
//
//            double currentPosition = getTurretDegrees();
//
//            // **CRITICAL: Clamp current position to valid range**
//            currentPosition = Math.max(MIN_LIMIT_DEGREES, Math.min(MAX_LIMIT_DEGREES, currentPosition));
//
//            double reference = getTurretTarget();
//
//            double error = angleErrorDegrees(reference, currentPosition);
//
//            // **STOP if we're at hard limit**
//            if (Math.abs(currentPosition - MIN_LIMIT_DEGREES) < 5 ||
//                    Math.abs(currentPosition - MAX_LIMIT_DEGREES) < 5) {
//                if (Math.signum(error) * Math.signum(currentPosition - (MIN_LIMIT_DEGREES + MAX_LIMIT_DEGREES)/2) > 0) {
//                    m.setPower(0); // Stop pushing against limit
//                    integralSum = 0; // Reset integral windup
//                    return;
//                }
//            }
//
//            double elapsedTime = timer.seconds();
//            if (elapsedTime <= 0) elapsedTime = 1e-3;
//            double errorChange = error - lastError;
//            double derivative = errorChange / elapsedTime;
//
//            integralSum += error * elapsedTime;
//            // **TIGHTER integral limits**
//            integralSum = Math.max(-20, Math.min(20, integralSum));
//
//            if (reference != lastReference || Math.abs(error) < 2) {
//                integralSum *= 0.5; // Decay integral when switching targets or on target
//            }
//
//            double output = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
//
//            // **OUTPUT LIMITS** - prevent excessive power near limits
//            if (Math.abs(error) > 1) {
//                output += Math.signum(error) * 0.05;
//            }
//
//            // **FINAL SAFETY CLAMP** - never exceed ±0.8 power
//            output = Math.max(-0.8, Math.min(0.8, output));
//            m.setPower(output);
//
//            Turret.error = error;
//            lastError = error;
//            lastReference = reference;
//            timer.reset();
//        }
//
//    public boolean atTarget() {
//        return Math.abs(getError()) < 1.0;
//    }
//
//    public void stop() {
//        m.setPower(0);
//        integralSum = 0;
//    }
//
//
//
//
//
//    public void automatic() { manual = false; }
//    public void on() { on = true; }
//    public void off() { on = false; }
//
//    public double getYaw() {
//        return normalizeAngle(Math.toRadians(getTurretDegrees()));
//    }
//
//
//    private double angleErrorDegrees(double target, double current) {
//        double error = target - current;
//        error = (error + 540.0) % 360.0 - 180.0;
//        return error;
//    }
//
//    public void setYaw(double radians) {
//        radians = normalizeAngle(radians);
//        setTurretTarget(Math.toDegrees(radians));
//    }
//
//    public void face(Pose targetPose, Pose robotPose) {
//        double dx = targetPose.getX() - robotPose.getX();
//        double dy = targetPose.getY() - robotPose.getY();
//
//        double targetAngle = Math.atan2(dy, dx);
//        double turretYaw = normalizeAngle(targetAngle - robotPose.getHeading());
//
//        FtcDashboard.getInstance().getTelemetry().addData("dx", "%.1f", dx);
//        FtcDashboard.getInstance().getTelemetry().addData("dy", "%.1f", dy);
//        FtcDashboard.getInstance().getTelemetry().addData("targetAngle", "%.2f rad", targetAngle);
//        FtcDashboard.getInstance().getTelemetry().addData("robotHeading", "%.2f rad", robotPose.getHeading());
//
//        setYaw(turretYaw);
//    }
//
//    public void incrementTurretRight() {
//        setTurretTarget(targetDegrees -= 20);
//    }
//    public void incrementTurretLeft() {
//        setTurretTarget(targetDegrees += 20);
//    }
//
//
//    public void resetTurret() {
//        setTurretTarget(0);
//    }
//
//    public static double normalizeAngle(double angleRadians) {
//        double angle = angleRadians % (Math.PI * 2D);
//        if (angle <= -Math.PI) angle += Math.PI * 2D;
//        if (angle > Math.PI) angle -= Math.PI * 2D;
//        return angle;
//    }
//
//    private double normalizeAngleDegrees(double angleDegrees) {
//        double angle = angleDegrees % 360.0;
//        if (angle > 180.0) angle -= 360.0;
//        if (angle <= -180.0) angle += 360.0;
//        return angle;
//    }
//
//    public double getError() { return error; }
//}


package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
//
@Config
@Configurable
public class Turret {
    public static double error = 0, power = 0, manualPower = 0;
    private final AnalogInput encoder;
    public final DcMotorEx m;
    public static double targetDegrees = 0;

    private double integralSum = 0;
    private double lastError = 0;
    private double lastReference = 0;
    private double previousFilterEstimate = 0;
    private double currentFilterEstimate = 0;

    private final double a = 0.8; // low-pass filter constant
    private final double maxIntegralSum = 45; // anti-windup limit (in degrees)

    public static double Kp = 0.0143;
    public static double Ki = 0.0;
    public static double Kd = 0.0001;


    private final ElapsedTime timer = new ElapsedTime();
    public static boolean on = true, manual = false;

    public Turret(HardwareMap hardwareMap) {
        encoder = hardwareMap.get(AnalogInput.class, "turretEncoder");
        m = hardwareMap.get(DcMotorEx.class, "turret");
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
// m.setDirection(DcMotor.Direction.REVERSE);
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }



    public static double ENCODER_ZERO_OFFSET_VOLTAGE = -3.192;
    public static double lastAngle = 0;
    public static double unwrappedAngle = 0;

    public double getFinalVoltage() {
        return encoder.getVoltage();
    }

    public double getTurretDegrees() {
        double voltage = encoder.getVoltage() + ENCODER_ZERO_OFFSET_VOLTAGE;
        double angle = (voltage / 3.3) * -360.0;
        return angle;
    }



    public double getVoltage() {
        return encoder.getVoltage();
    }
    public double getTurretTarget() { return targetDegrees; }
    public void setTurretTarget(double degrees) {
        targetDegrees = Math.max(-145, Math.min(125, degrees));
    }

//    public void periodic() {
//        if (!on) {
//            m.setPower(0);
//            return;
//        }
//        if (manual) {
//            m.setPower(manualPower);
//            return;
//        }
//
//        double currentPosition = getTurretDegrees();
//        double reference = getTurretTarget();
//
//        double error = angleErrorDegrees(reference, currentPosition);
//
//        double elapsedTime = timer.seconds();
//        if (elapsedTime <= 0) elapsedTime = 1e-3;
//        double errorChange = error - lastError;
//
//// currentFilterEstimate = (a * previousFilterEstimate) + ((1 - a) * errorChange);
//// previousFilterEstimate = currentFilterEstimate;
////
//// double derivative = currentFilterEstimate / elapsedTime;
//        double derivative = errorChange / elapsedTime;
//
//        integralSum += error * elapsedTime;
//        integralSum = Math.max(-maxIntegralSum, Math.min(maxIntegralSum, integralSum));
//
//        if (reference != lastReference) {
//            integralSum = 0;
//        }
//
//        double output = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
//
//        if (Math.abs(error) > 1)
//            output += Math.signum(output) * 0.05;
//
//        output*=1;
//        m.setPower(Math.max(-1, Math.min(1, output)));
//
//        Turret.error = error;
//        lastError = error;
//        lastReference = reference;
//        timer.reset();
//    }
//public void periodic() {
//    if (!on) {
//        m.setPower(0);
//        return;
//    }
//    if (manual) {
//        m.setPower(manualPower);
//        return;
//    }
//
//    double currentPosition = getTurretDegrees();
//    double reference = getTurretTarget();
//
//    // Simple, non‑wrapping error
//    double error = reference - currentPosition;
//
//    double elapsedTime = timer.seconds();
//    if (elapsedTime <= 0) elapsedTime = 1e-3;
//
//    double errorChange = error - lastError;
//    double derivative = errorChange / elapsedTime;
//
//    integralSum += error * elapsedTime;
//    integralSum = Math.max(-maxIntegralSum, Math.min(maxIntegralSum, integralSum));
//
//    if (reference != lastReference) {
//        integralSum = 0;
//    }
//
//    double output = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
//
//    if (Math.abs(error) > 1)
//        output += Math.signum(output) * 0.05;
//
//    m.setPower(Math.max(-1, Math.min(1, output)));
//
//    Turret.error = error;
//    lastError = error;
//    lastReference = reference;
//    timer.reset();
//}

public void periodic() {
    if (!on) {
        m.setPower(0);
        return;
    }
    if (manual) {
        m.setPower(manualPower);
        return;
    }

    double currentPosition = getTurretDegrees();
    double reference = getTurretTarget();

    double error = angleErrorDegrees(reference, currentPosition);

    double elapsedTime = timer.seconds();
    if (elapsedTime <= 0) elapsedTime = 1e-3;

    double errorChange = error - lastError;
    double derivative = errorChange / elapsedTime;

    integralSum += error * elapsedTime;
    integralSum = Math.max(-maxIntegralSum, Math.min(maxIntegralSum, integralSum));

    if (reference != lastReference) {
        integralSum = 0;
    }

    double output = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

    if (Math.abs(error) > 1)
        output += Math.signum(error) * 0.05;

    m.setPower(Math.max(-0.9, Math.min(0.9, output)));  // Lower power for stability

    Turret.error = error;
    lastError = error;
    lastReference = reference;
    timer.reset();
}


    public void automatic() { manual = false; }
    public void on() { on = true; }
    public void off() { on = false; }

    public double getYaw() {
        return normalizeAngle(Math.toRadians(getTurretDegrees()));
    }


    private double angleErrorDegrees(double target, double current) {
        double error = target - current;
        error = (error + 540.0) % 360.0 - 180.0;
        return error;
    }

    public void setYaw(double radians) {
        radians = normalizeAngle(radians);
        setTurretTarget(Math.toDegrees(radians));
    }

    public void face(Pose targetPose, Pose robotPose) {
        double dx = targetPose.getX() - robotPose.getX();
        double dy = targetPose.getY() - robotPose.getY();

        double targetAngle = Math.atan2(dy, dx);
        double turretYaw = normalizeAngle(targetAngle - robotPose.getHeading());

        FtcDashboard.getInstance().getTelemetry().addData("dx", "%.1f", dx);
        FtcDashboard.getInstance().getTelemetry().addData("dy", "%.1f", dy);
        FtcDashboard.getInstance().getTelemetry().addData("targetAngle", "%.2f rad", targetAngle);
        FtcDashboard.getInstance().getTelemetry().addData("robotHeading", "%.2f rad", robotPose.getHeading());

        setYaw(turretYaw);
    }
// SOTM
//    public void face(Pose targetPose, Pose robotPose, Vector botVelocity) {  // Add velocity
//        double dx = targetPose.getX() - robotPose.getX();
//        double dy = targetPose.getY() - robotPose.getY();
//
//        double targetAngle = Math.atan2(dy, dx);
//        double turretYaw = normalizeAngle(targetAngle - robotPose.getHeading());
//
//        double leadK = 0.25;  // Tune: 0.15-0.4s (start 0.25)
//
//        double losAngle = targetAngle;
//        double velPerp = -(botVelocity.getXComponent() * Math.sin(losAngle) - botVelocity.getYComponent() * Math.cos(losAngle));
//
//        // Lead angle (radians)
//        double leadAngle = (velPerp * leadK) / Math.hypot(dx, dy);  // / distance scales properly
//
//        turretYaw += leadAngle;
//        turretYaw = normalizeAngle(turretYaw);
//
//        setYaw(turretYaw);
//    }


    public void incrementTurretRight() {
        setTurretTarget(targetDegrees -= 20);
    }
    public void incrementTurretLeft() {
        setTurretTarget(targetDegrees += 20);
    }


    public void resetTurret() {
        setTurretTarget(0);
    }

    public static double normalizeAngle(double angleRadians) {
        double angle = angleRadians % (Math.PI * 2D);
        if (angle <= -Math.PI) angle += Math.PI * 2D;
        if (angle > Math.PI) angle -= Math.PI * 2D;
        return angle;
    }

    private double normalizeAngleDegrees(double angleDegrees) {
        double angle = angleDegrees % 360.0;
        if (angle > 180.0) angle -= 360.0;
        if (angle <= -180.0) angle += 360.0;
        return angle;
    }

    public double getError() { return error; }
}