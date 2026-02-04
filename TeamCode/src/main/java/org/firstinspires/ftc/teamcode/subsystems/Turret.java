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

@Configurable
public class Turret {
    public static double error = 0, power = 0, manualPower = 0;
    public static double rpt = 0.00653;

    public final DcMotorEx m;
    private PIDFController primaryPID, secondaryPID;
    public static double target = 0;
    public static double pidfSwitch = 5;
    public static double kp = 0.004, kf = 0.0001, kd = 0.000, sp = .008, sf = 0, sd = 0.0001;

    public static boolean on = true, manual = false;



    public Turret(HardwareMap hardwareMap) {
        m = hardwareMap.get(DcMotorEx.class, "turret");
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        primaryPID = new PIDFController(new PIDFCoefficients(kp, 0, kd, kf));
        secondaryPID = new PIDFController(new PIDFCoefficients(sp, 0, sd, sf));
    }

    public void setTurretTarget(double ticks) {
        target = Math.max(-350, Math.min(345, ticks));
    }


    public double getTurretTarget() { return target; }
    public double getTurret() { return m.getCurrentPosition(); }

    public void periodic() {
        if (!on) { m.setPower(0); return; }

        if (manual) { m.setPower(manualPower); return; }

        primaryPID.setCoefficients(new PIDFCoefficients(kp, 0, kd, kf));
        secondaryPID.setCoefficients(new PIDFCoefficients(sp, 0, sd, sf));

        error = getTurretTarget() - getTurret();
        if (Math.abs(error) > pidfSwitch) {
            primaryPID.updateError(error);
            primaryPID.updateFeedForwardInput(Math.signum(error));
            power = primaryPID.run();
        } else {
            secondaryPID.updateError(error);
            power = secondaryPID.run();
        }

        m.setPower(power);
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
