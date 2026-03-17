////package org.firstinspires.ftc.teamcode.subsystems;
////
////
////import com.acmerobotics.dashboard.config.Config;
////import com.bylazar.configurables.annotations.Configurable;
////import com.qualcomm.hardware.lynx.LynxModule;
////import com.qualcomm.robotcore.hardware.DcMotor;
////import com.qualcomm.robotcore.hardware.HardwareMap;
////import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
////
////import java.util.List;
////
////@Configurable
////@Config
////public class Outtake {
////
////    private final MotorEx left, right;
////    private final List<LynxModule> allHubs;
////    // target RPMs
////    public static double HIGH_RPM = 4300;
////    public static double LOW_RPM  = 3700;
////    public static double HIGH_HIGH_RPM = 4600;
////    public static double TICKS_PER_REV = 28.0;
////
////    public static double kP = 0.000061;
////    public static double kI = 0.0;
////    public static double kD = 0.00001;
////
////    // Feedforward (kS + kV + kA) are unitless motor model gains
////    public static double kS = 0;
////    public static double kV = 0.00038;
////    public static double kA = 0.0;   // start at 0 unless you need accel comp
////
////
////    private static final double RPM_FILTER_ALPHA = 0.6;
////
////    private double leftFilteredRPM = 0.0;
////    private double rightFilteredRPM = 0.0;
////    private boolean rpmFilterInitialized = false;
////
////
////    public double targetRPM = 0;
////    private boolean enabled = false;
////    private long stableStartTime = 0;
////
////    private double lowPass(double previous, double current) {
////        return RPM_FILTER_ALPHA * previous + (1.0 - RPM_FILTER_ALPHA) * current;
////    }
////
////
////    public Outtake(HardwareMap hw) {
////        allHubs = hw.getAll(LynxModule.class);
////        for (LynxModule hub : allHubs) {
////            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
////        }
////
////        left  = new MotorEx(hw, "outtakeLeft");
////        right = new MotorEx(hw, "outtakeRight");
////
////        left.setInverted(true);
////        right.setInverted(false);
////
////        // motor run mode: velocity controlled
////        left.setRunMode(MotorEx.RunMode.VelocityControl);
////        right.setRunMode(MotorEx.RunMode.VelocityControl);
////
////        // configure coeffs once
////        updateVeloPIDF();
////
////        stop();
////    }
////
////    private void updateVeloPIDF() {
////        // configure PID
////        left.setVeloCoefficients(kP, kI, kD);
////        right.setVeloCoefficients(kP, kI, kD);
////
////        // configure feedforward
////        if (kA == 0.0) {
////            left.setFeedforwardCoefficients(kS, kV);
////            right.setFeedforwardCoefficients(kS, kV);
////        } else {
////            left.setFeedforwardCoefficients(kS, kV, kA);
////            right.setFeedforwardCoefficients(kS, kV, kA);
////        }
////    }
////
////    public double getTarget() {return HIGH_RPM;}
////
////    public void shootHigh() { setTargetRPM(HIGH_RPM); }
////    public void shootLow()  { setTargetRPM(LOW_RPM); }
////    public void shootDoubleHigh() {setTargetRPM(HIGH_HIGH_RPM);}
////
////    public void stop() {
////        enabled = false;
////        targetRPM = 0;
////        stableStartTime = 0;
////
////        rpmFilterInitialized = false;
////        leftFilteredRPM = 0.0;
////        rightFilteredRPM = 0.0;
////
////        left.setRunMode(MotorEx.RunMode.RawPower);
////        right.setRunMode(MotorEx.RunMode.RawPower);
////        left.set(0);
////        right.set(0);
////    }
////
////
////    public void setTargetRPM(double rpm) {
////        targetRPM = rpm;
////        enabled = rpm != 0;
////        stableStartTime = 0;
////
////        if (!enabled) {
////            stop();
////            return;
////        }
////        // ensure velocity mode is active
////        left.setRunMode(MotorEx.RunMode.VelocityControl);
////        right.setRunMode(MotorEx.RunMode.VelocityControl);
////
////        double ticksPerSec = rpm * TICKS_PER_REV / 60.0;
////
////        left.set(ticksPerSec);
////        right.set(ticksPerSec);
////    }
////
////    public void periodic() {
////        if (!enabled || targetRPM == 0) {
////            // keep motors stopped if not enabled
////            left.setRunMode(MotorEx.RunMode.RawPower);
////            right.setRunMode(MotorEx.RunMode.RawPower);
////            left.set(0);
////            right.set(0);
////            return;
////        }
////
////        updateVeloPIDF();
////
////        double ticksPerSec = targetRPM * TICKS_PER_REV / 60.0;
////        left.set(ticksPerSec);
////        right.set(ticksPerSec);
////    }
////
////
////    public double getRPMLeft() {
////        double rawRPM =
////                Math.abs(left.getVelocity() * 60.0 / TICKS_PER_REV);
////
////        if (!rpmFilterInitialized) {
////            leftFilteredRPM = rawRPM;
////            rpmFilterInitialized = true;
////            return rawRPM;
////        }
////
////        leftFilteredRPM = lowPass(leftFilteredRPM, rawRPM);
////        return leftFilteredRPM;
////    }
////
////
////    public double getRPMRight() {
////        double rawRPM =
////                Math.abs(right.getVelocity() * 60.0 / TICKS_PER_REV);
////
////        if (!rpmFilterInitialized) {
////            rightFilteredRPM = rawRPM;
////            rpmFilterInitialized = true;
////            return rawRPM;
////        }
////
////        rightFilteredRPM = lowPass(rightFilteredRPM, rawRPM);
////        return rightFilteredRPM;
////    }
////
////
////    public double getTickLeft()  { return left.getVelocity(); }
////    public double getTickRight() { return right.getVelocity(); }
////    public boolean isEnabled()   { return enabled; }
////}
//
//package org.firstinspires.ftc.teamcode.subsystems;
//
//import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.KalmanFilter;
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.seattlesolvers.solverslib.controller.PIDFController;
//
//@Config
//public class Outtake {
//    public static double HIGH_RPM = 4300;
//    public static double LOW_RPM = 3700;
//    public static double HIGH_HIGH_RPM = 4600;
//    public static double TICKS_PER_REV = 28.0;
//
//    public static double kP = 0.0012;
//    public static double kI = 0.001;
//    public static double kD = 0.0001;
//    public static double kF = 0.00022;
//
//    // Kalman values
//    public static double Q = 0.25;
//    public static double R = 7;
//    public static int N = 5;
//
//    private DcMotorEx left, right;
//    private double targetRPM = 0;
//    private double estimatedVelocity = 0;
//
//    private PIDFController pidfController = new PIDFController(kP, kI, kD, kF);
//    private KalmanFilter filter = new KalmanFilter(Q, R, N);
//
//    public Outtake(HardwareMap hw) {
//        left = hw.get(DcMotorEx.class, "outtakeLeft");
//        right = hw.get(DcMotorEx.class, "outtakeRight");
//
//        left.setDirection(DcMotorSimple.Direction.REVERSE);
//        right.setDirection(DcMotorSimple.Direction.FORWARD);
//
//        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//
//        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        stop();
//    }
//
//    public void periodic() {
//        double velocityNoisy = getVelocityUnfiltered(); // Right motor only
//        estimatedVelocity = filter.estimate(velocityNoisy);
//
//        double output = pidfController.calculate(estimatedVelocity, targetRPM);
//        output = Math.max(-1.0, Math.min(1.0, output));
//
//        if (Math.abs(targetRPM) < 1e-6) {
//            pidfController.reset();
//            left.setPower(0);
//            right.setPower(0);
//        } else {
//            left.setPower(output);
//            right.setPower(output);
//        }
//    }
//
//    private double getVelocityUnfiltered() {
//        return (right.getVelocity() / TICKS_PER_REV) * 60;
//    }
//
//    public void setTargetRPM(double rpm) {
//        targetRPM = rpm;
//    }
//
//    public void shootHigh() { setTargetRPM(HIGH_RPM); }
//    public void shootLow() { setTargetRPM(LOW_RPM); }
//    public void shootDoubleHigh() { setTargetRPM(HIGH_HIGH_RPM); }
//    public void stop() { setTargetRPM(0); }
//
//    // For RGB indicator (updated for single value)
//    public double getEstimatedRPM() { return estimatedVelocity; }
//    public double getTarget() { return targetRPM; }
//    public boolean isEnabled() { return targetRPM != 0; }
//}

package org.firstinspires.ftc.teamcode.subsystems;

import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.KalmanFilter;
//import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.qualcomm.hardware.lynx.LynxModule;

import java.util.List;

@Config
@Configurable
public class Outtake {
    public static double HIGH_RPM = 4200;
    public static double LOW_RPM = 3700;
    public static double HIGH_HIGH_RPM = 4600;
    public static double TICKS_PER_REV = 28.0;

    public static double kP = 0.0012;
    public static double kI = 0.001;
    public static double kD = 0.0001;
    public static double kF = 0.00022;

    // Kalman values
    public static double Q = 0.25;
    public static double R = 7;
    public static int N = 5;

    private DcMotorEx left, right;
    private final List<LynxModule> allHubs;

    public double targetRPM = 0;
    private double estimatedVelocity = 0;

    private PIDFController pidfController = new PIDFController(kP, kI, kD, kF);
    private KalmanFilter filter = new KalmanFilter(Q, R, N);

    // RIGHT MOTOR ONLY for all readings/telemetry
    private double rightFilteredRPM = 0.0;
    private boolean rpmFilterInitialized = false;
    private static final double RPM_FILTER_ALPHA = 0.6;

    public Outtake(HardwareMap hw) {
        allHubs = hw.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        left = hw.get(DcMotorEx.class, "outtakeLeft");
        right = hw.get(DcMotorEx.class, "outtakeRight");

        left.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setDirection(DcMotorSimple.Direction.FORWARD);

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        stop();
    }

    public void periodic() {
        // RIGHT MOTOR ONLY readings
        double rightRawRPM = Math.abs(getRawRPM(right));

        if (!rpmFilterInitialized) {
            rightFilteredRPM = rightRawRPM;
            rpmFilterInitialized = true;
        } else {
            rightFilteredRPM = lowPass(rightFilteredRPM, rightRawRPM);
        }

        // Kalman + PIDF control (right motor feedback only)
        double velocityNoisy = getRawRPM(right);
        estimatedVelocity = filter.estimate(velocityNoisy);

        double output = pidfController.calculate(estimatedVelocity, targetRPM);
        output = Math.max(-1.0, Math.min(1.0, output));

        if (Math.abs(targetRPM) < 1e-6) {
            pidfController.reset();
            left.setPower(0);
            right.setPower(0);
        } else {
            left.setPower(output);
            right.setPower(output);
        }
    }

    private double lowPass(double previous, double current) {
        return RPM_FILTER_ALPHA * previous + (1.0 - RPM_FILTER_ALPHA) * current;
    }

    private double getRawRPM(DcMotorEx motor) {
        return (motor.getVelocity() / TICKS_PER_REV) * 60;
    }

    // ALL METHODS USE RIGHT MOTOR READINGS ONLY
    public double getRPMLeft() { return rightFilteredRPM; }   // Returns RIGHT
    public double getRPMRight() { return rightFilteredRPM; }  // Returns RIGHT
    public double getTickLeft() { return right.getVelocity(); }   // Returns RIGHT
    public double getTickRight() { return right.getVelocity(); }  // Returns RIGHT
    public boolean isEnabled() { return targetRPM != 0; }
    public double getTarget() { return targetRPM; }

    public void setTargetRPM(double rpm) { targetRPM = rpm; }
    public void shootHigh() { setTargetRPM(HIGH_RPM); }
    public void shootLow() { setTargetRPM(LOW_RPM); }
    public void autonLow() { setTargetRPM(3600); }
    public void autonLowEnd() { setTargetRPM(3500); }
    public void shootDoubleHigh() { setTargetRPM(HIGH_HIGH_RPM); }
    public void stop() { setTargetRPM(0); }
}
