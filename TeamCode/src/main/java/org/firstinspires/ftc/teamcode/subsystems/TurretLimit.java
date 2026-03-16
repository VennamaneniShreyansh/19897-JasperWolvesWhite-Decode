package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class TurretLimit {

    public static double error = 0, power = 0, manualPower = 0;

    private final AnalogInput encoder;
    public final DcMotorEx m;

    public static double targetDegrees = 0;

    public static double MIN_LIMIT = -170;
    public static double MAX_LIMIT = 135;

    // Soft zone slows PID near edges
    public static double SOFT_LIMIT_ZONE = 10;

    private double integralSum = 0;
    private double lastError = 0;
    private double lastReference = 0;

    private double previousFilterEstimate = 0;

    public static double Kp = 0.08;
    public static double Ki = 0.0;
    public static double Kd = 0.00001;

    private final ElapsedTime timer = new ElapsedTime();

    public static boolean on = true, manual = false;

    public TurretLimit(HardwareMap hardwareMap) {

        encoder = hardwareMap.get(AnalogInput.class, "turretEncoder");

        m = hardwareMap.get(DcMotorEx.class, "turret");

        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public static double ENCODER_ZERO_OFFSET_DEG = 1.215;

    public double getTurretDegrees() {

        double voltage = encoder.getVoltage();

        double angle = (voltage / 3.3) * 360.0;

        angle -= (ENCODER_ZERO_OFFSET_DEG / 3.3) * 360.0;

        return angle;
    }

    public double getVoltage() {
        return encoder.getVoltage();
    }

    public double getTurretTarget() {
        return targetDegrees;
    }

    public void setTurretTarget(double degrees) {

        targetDegrees = Math.max(MIN_LIMIT, Math.min(MAX_LIMIT, degrees));
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

        double currentPosition = getTurretDegrees();

        // Clamp reference to wiring limits
        double reference = Math.max(MIN_LIMIT, Math.min(MAX_LIMIT, getTurretTarget()));

        double error = angleErrorDegrees(reference, currentPosition);

        double elapsedTime = timer.seconds();
        if (elapsedTime <= 0) elapsedTime = 1e-3;

        double errorChange = error - lastError;

        double a = 0.8;
        double currentFilterEstimate = (a * previousFilterEstimate) +
                ((1 - a) * errorChange);

        previousFilterEstimate = currentFilterEstimate;

        double derivative = currentFilterEstimate / elapsedTime;

        integralSum += error * elapsedTime;

        double maxIntegralSum = 45;
        integralSum = Math.max(-maxIntegralSum, Math.min(maxIntegralSum, integralSum));

        if (Math.abs(reference - lastReference) > 0.01) {
            integralSum = 0;
        }

        double output =
                (Kp * error) +
                        (Ki * integralSum) +
                        (Kd * derivative);

        if (currentPosition >= MAX_LIMIT && output > 0) {

            output = 0;
            integralSum = 0;

        }

        if (currentPosition <= MIN_LIMIT && output < 0) {

            output = 0;
            integralSum = 0;

        }

        if (currentPosition > MAX_LIMIT - SOFT_LIMIT_ZONE && output > 0) {

            double scale =
                    (MAX_LIMIT - currentPosition) / SOFT_LIMIT_ZONE;

            output *= Math.max(0, scale);

        }

        if (currentPosition < MIN_LIMIT + SOFT_LIMIT_ZONE && output < 0) {

            double scale =
                    (currentPosition - MIN_LIMIT) / SOFT_LIMIT_ZONE;

            output *= Math.max(0, scale);

        }

        power = Math.max(-1, Math.min(1, output));

        m.setPower(power);

        Turret.error = error;

        lastError = error;
        lastReference = reference;

        timer.reset();
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

        return normalizeAngle(Math.toRadians(getTurretDegrees()));
    }

    public void setYaw(double radians) {

        radians = normalizeAngle(radians);

        setTurretTarget(Math.toDegrees(radians));
    }

    public void faceWithVelocityComp(
            Pose targetPose,
            Pose robotPose,
            Pose robotVelocity,
            double projectileSpeed
    ) {

        double dx = targetPose.getX() - robotPose.getX();
        double dy = targetPose.getY() - robotPose.getY();

        double distance = Math.hypot(dx, dy);
        if (distance < 1e-6) return;

        // Base angle to goal
        double targetAngle = Math.atan2(dy, dx);

        // Robot velocity
        double vx = robotVelocity.getX();
        double vy = robotVelocity.getY();

        double ux = dx / distance;
        double uy = dy / distance;

        double px = -uy;
        double py = ux;

        double vPer = vx * px + vy * py;

        // Projectile airtime approximation
        double airTime = distance / projectileSpeed;

        // Sideways drift
        double drift = vPer * airTime;

        // Angle offset needed to compensate
        double offsetAngle = Math.atan2(drift, distance);

        double compensatedAngle = targetAngle + offsetAngle;

        double turretYaw = normalizeAngle(compensatedAngle - robotPose.getHeading());

        setYaw(turretYaw);
    }

    private double angleErrorDegrees(double target, double current) {

        double error = target - current;

        error = (error + 540.0) % 360.0 - 180.0;

        return error;
    }

    public void incrementTurretRight() {

        setTurretTarget(targetDegrees - 20);

    }

    public void incrementTurretLeft() {

        setTurretTarget(targetDegrees + 20);

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

    public double getError() {

        return error;

    }
}