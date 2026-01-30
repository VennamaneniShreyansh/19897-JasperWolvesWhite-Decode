//package org.firstinspires.ftc.teamcode.subsystems;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//@Configurable
//public class Outtake {
//
//    private final DcMotorEx left, right;
//
//    public static double HIGH_RPM = 4800;
//    public static double LOW_RPM  = 4300;
//    public static double TICKS_PER_REV = 28;
//
//    public static double kP = 0.19;    // Start 1.0–2.0
//    public static double kI = 0.0;    // Generally 0 for velocity
//    public static double kD = 0.04;    // For damping oscillation
//    public static double kF = 14.25;  // Feedforward-> tune first
//
//    public double targetRPM = 0;
//    private double lastTargetTicks = 0;
//    private boolean enabled = false;
//    private long stableStartTime = 0;
//
//    public Outtake(HardwareMap hw) {
//        left  = hw.get(DcMotorEx.class, "outtakeLeft");
//        right = hw.get(DcMotorEx.class, "outtakeRight");
//
//        // Recommended: reverse one motor then keep power positive
//        left.setDirection(DcMotor.Direction.REVERSE);
//        right.setDirection(DcMotor.Direction.FORWARD);
//
//        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        updatePIDF();
//        stop();
//    }
//
//    private void updatePIDF() {
//        left.setVelocityPIDFCoefficients(kP, kI, kD, kF);
//        right.setVelocityPIDFCoefficients(kP, kI, kD, kF);
//    }
//
//    public void shootHigh() { setTargetRPM(HIGH_RPM); }
//    public void shootLow()  { setTargetRPM(LOW_RPM); }
//
//
//
//    public void stop() {
//        enabled = false;
//        targetRPM = 0;
//        lastTargetTicks = 0;
//        left.setVelocity(0);
//        right.setVelocity(0);
//        stableStartTime = 0;
//    }
//
//    public void setTargetRPM(double rpm) {
//        targetRPM = rpm;
//        enabled = true;
//
//        double ticksPerSec = rpm * TICKS_PER_REV / 60.0;
//        if (Math.abs(ticksPerSec - lastTargetTicks) > 1) {
//            left.setVelocity(ticksPerSec);
//            right.setVelocity(ticksPerSec);
//            lastTargetTicks = ticksPerSec;
//        }
//        stableStartTime = 0;
//    }
//
//    public void periodic() {
//        updatePIDF();  // allow live tuning
//
//        if (!enabled || targetRPM == 0) {
//            if (Math.abs(lastTargetTicks) > 1) {
//                stop();
//            }
//        }
//    }
//
//    public boolean atTarget() {
//        double leftRPM  = getRPMLeft();
//        double rightRPM = getRPMRight();
//        boolean steady = Math.abs(leftRPM - targetRPM) < 50 && Math.abs(rightRPM - targetRPM) < 50;
//
//        if (steady) {
//            if (stableStartTime == 0) stableStartTime = System.currentTimeMillis();
//            return System.currentTimeMillis() - stableStartTime > 50;
//        } else {
//            stableStartTime = 0;
//            return false;
//        }
//    }
//
//    public double getRPMLeft()  { return Math.abs(left.getVelocity() * 60 / TICKS_PER_REV); }
//    public double getRPMRight() { return Math.abs(right.getVelocity() * 60 / TICKS_PER_REV); }
//    public double getTickLeft() { return left.getVelocity(); }
//    public double getTickRight(){ return right.getVelocity(); }
//
//    public boolean isEnabled() { return enabled; }
//}


package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
//
//@Configurable
//public class Outtake {
//
//    private final DcMotorEx left, right;
//
//    public static double HIGH_RPM = 4800;
//    public static double LOW_RPM  = 4300;
//    public static double TICKS_PER_REV = 28;
//
//    // Manual PIDF (configurable)
//    public static double kP = 0.19;
//    public static double kI = 0.0;
//    public static double kD = 0.04;
//    public static double kF = 14.25;
//
//    // Safety / tolerance
//    public static double RPM_TOLERANCE = 50.0;
//
//    public double targetRPM = 0;
//    private double lastError = 0;
//    private double integral = 0;
//    private long lastTime = 0;
//
//    private boolean enabled = true;
//    private long stableStartTime = 0;
//
//    public Outtake(HardwareMap hw) {
//        left  = hw.get(DcMotorEx.class, "outtakeLeft");
//        right = hw.get(DcMotorEx.class, "outtakeRight");
//
//        left.setDirection(DcMotor.Direction.REVERSE);
//        right.setDirection(DcMotor.Direction.FORWARD);
//
//        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        stop();
//    }
//
//    public void shootHigh() { setTargetRPM(HIGH_RPM); }
//    public void shootLow()  { setTargetRPM(LOW_RPM); }
//
//    public void stop() {
//        enabled = false;
//        targetRPM = 0;
//        lastError = 0;
//        integral = 0;
//        lastTime = 0;
//        stableStartTime = 0;
//        left.setPower(0);
//        right.setPower(0);
//    }
//
//    public void setTargetRPM(double rpm) {
//        targetRPM = rpm;
//        enabled = rpm != 0;
//        lastError = 0;
//        integral = 0;
//        lastTime = 0;
//        stableStartTime = 0;
//    }
//
//    // Call this every loop from your OpMode
//    public void periodic() {
//        if (!enabled || targetRPM == 0) {
//            // already off
//            return;
//        }
//
//        double leftRPM  = getRPMLeft();
//        double rightRPM = getRPMRight();
//        double currentRPM = (leftRPM + rightRPM) / 2.0;
//
//        double error = targetRPM - currentRPM;
//        long now = System.nanoTime();
//        double dt = (lastTime == 0) ? 0 : (now - lastTime) / 1e9;
//        lastTime = now;
//
//        if (dt > 0) {
//            integral += error * dt;
//            double derivative = (error - lastError) / dt;
//            lastError = error;
//
//            // Manual PIDF output -> power
//            double ff = kF * targetRPM;    // you will tune this so it's in the right scale
//            double output = ff + kP * error + kI * integral + kD * derivative;
//
//            // Clamp power
//            output = Math.max(-1.0, Math.min(1.0, output));
//
//            left.setPower(output);
//            right.setPower(output);
//        }
//
//        // Auto‑disable when within tolerance
//        if (Math.abs(error) <= RPM_TOLERANCE) {
//            // consider it stable, stop motor so it holds speed by inertia
//            stop();
//        }
//    }
//
//    public boolean atTarget() {
//        double leftRPM  = getRPMLeft();
//        double rightRPM = getRPMRight();
//        boolean steady =
//                Math.abs(leftRPM - targetRPM) <= RPM_TOLERANCE &&
//                        Math.abs(rightRPM - targetRPM) <= RPM_TOLERANCE;
//
//        if (steady) {
//            if (stableStartTime == 0) stableStartTime = System.currentTimeMillis();
//            return System.currentTimeMillis() - stableStartTime > 50;
//        } else {
//            stableStartTime = 0;
//            return false;
//        }
//    }
//
//    public double getRPMLeft()  { return Math.abs(left.getVelocity() * 60.0 / TICKS_PER_REV); }
//    public double getRPMRight() { return Math.abs(right.getVelocity() * 60.0 / TICKS_PER_REV); }
//    public double getTickLeft() { return left.getVelocity(); }
//    public double getTickRight(){ return right.getVelocity(); }
//
//    public boolean isEnabled() { return enabled; }
//}



@Configurable
public class Outtake {

    private final DcMotorEx left, right;

    public static double HIGH_RPM = 4400;
    public static double LOW_RPM  = 5000;
    public static double TICKS_PER_REV = 28;

    // Manual power PIDF (configurable)
    public static double kP = 0.0005;  // RPM error → power (tune small)
    public static double kI = 0.00001; // RPM error * sec → power
    public static double kD = 0.0002;  // RPM/sec → power
    public static double kF = 0.85;    // base power (0.7–1.0, tune first!)

    public static double RPM_TOLERANCE = 50.0;

    public double targetRPM = 0;
    private double lastErrorL = 0, lastErrorR = 0;
    private double integralL = 0, integralR = 0;
    private long lastTime = 0;

    private boolean enabled = false;
    private long stableStartTime = 0;

    public Outtake(HardwareMap hw) {
        left  = hw.get(DcMotorEx.class, "outtakeLeft");
        right = hw.get(DcMotorEx.class, "outtakeRight");

        left.setDirection(DcMotor.Direction.REVERSE);
        right.setDirection(DcMotor.Direction.FORWARD);

        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        stop();
    }

    public void shootHigh() { setTargetRPM(HIGH_RPM); }
    public void shootLow()  { setTargetRPM(LOW_RPM); }

    public void stop() {
        enabled = false;
        targetRPM = 0;
        lastErrorL = lastErrorR = 0;
        integralL = integralR = 0;
        lastTime = 0;
        stableStartTime = 0;
        left.setPower(0);
        right.setPower(0);
    }

    public void setTargetRPM(double rpm) {
        targetRPM = rpm;
        enabled = rpm != 0;
        lastErrorL = lastErrorR = 0;
        integralL = integralR = 0;
        lastTime = 0;
        stableStartTime = 0;
    }

    public void periodic() {
        if (!enabled || targetRPM == 0) {
            left.setPower(0);
            right.setPower(0);
            return;
        }

        long now = System.nanoTime();
        double dt = (lastTime == 0) ? 0.02 : (now - lastTime) / 1e9;
        lastTime = now;

        double leftVel  = getRPMLeft();
        double rightVel = getRPMRight();

        double errorL = targetRPM - leftVel;
        double errorR = targetRPM - rightVel;

        if (Math.abs(errorL) <= RPM_TOLERANCE && Math.abs(errorR) <= RPM_TOLERANCE) {
//            left.setPower(0);
//            right.setPower(0);
            return;
        }

        integralL += errorL * dt;
        integralR += errorR * dt;

        double derivL = (errorL - lastErrorL) / Math.max(dt, 0.001);
        double derivR = (errorR - lastErrorR) / Math.max(dt, 0.001);
        lastErrorL = errorL;
        lastErrorR = errorR;
        integralL = Math.max(-1.0, Math.min(1.0, integralL));
        integralR = Math.max(-1.0, Math.min(1.0, integralR));
        double powerL = kF + kP * errorL + kI * integralL + kD * derivL;
        double powerR = kF + kP * errorR + kI * integralR + kD * derivR;

        left.setPower(Math.max(0, Math.min(1.0, powerL)));
        right.setPower(Math.max(0, Math.min(1.0, powerR)));
    }


    public boolean atTarget() {
        double leftRPM  = getRPMLeft();
        double rightRPM = getRPMRight();
        boolean steady =
                Math.abs(leftRPM - targetRPM) <= RPM_TOLERANCE &&
                        Math.abs(rightRPM - targetRPM) <= RPM_TOLERANCE;

        if (steady) {
            if (stableStartTime == 0) stableStartTime = System.currentTimeMillis();
            return System.currentTimeMillis() - stableStartTime > 50;
        } else {
            stableStartTime = 0;
            return false;
        }
    }

    public double getRPMLeft()  { return Math.abs(left.getVelocity()  * 60.0 / TICKS_PER_REV); }
    public double getRPMRight() { return Math.abs(right.getVelocity() * 60.0 / TICKS_PER_REV); }
    public double getTickLeft() { return left.getVelocity(); }
    public double getTickRight(){ return right.getVelocity(); }
    public boolean isEnabled()  { return enabled; }
}
