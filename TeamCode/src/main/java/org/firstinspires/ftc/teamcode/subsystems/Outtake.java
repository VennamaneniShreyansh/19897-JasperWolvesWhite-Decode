//package org.firstinspires.ftc.teamcode.subsystems;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//@Configurable
//public class Outtake {
//
//    private final DcMotorEx left, right;
//
//    public static double HIGH_RPM = 4500;
//    public static double LOW_RPM  = 3000;
//    public static double TICKS_PER_REV = 28;
//
//    public static double kP = 20, kI = 0, kD = 0;
//    public static double kF = 32767.0 / (6000 * TICKS_PER_REV);
//
//    public double targetRPM = 0;
//    public boolean enabled = false;
//
//    public Outtake(HardwareMap hw) {
//        left  = hw.get(DcMotorEx.class, "outtakeLeft");
//        right = hw.get(DcMotorEx.class, "outtakeRight");
//
//        left.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
////        right.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//
//        left.setVelocityPIDFCoefficients(kP, kI, kD, kF);
//        right.setVelocityPIDFCoefficients(kP, kI, kD, kF);
//    }
//
//    public void shootHigh() {
//        targetRPM = HIGH_RPM;
//        enabled = true;
//    }
//
//    public void shootLow() {
//        targetRPM = LOW_RPM;
//        enabled = true;
//    }
//
//    public double getRPMLeft() {
//        return getRPM(left);
//    }
//    public double getRPMRight() {
//        return getRPM(right);
//    }
//
//    public double getTickLeft() {
//        return left.getVelocity();
//    }
//    public double getTickRight() {
//        return right.getVelocity();
//    }
//    public void stop() {
//        enabled = false;
//        left.setVelocity(0);
//        right.setVelocity(0);
//    }
//
//    public void periodic() {
//        if (!enabled) return;
//
//        double ticks = targetRPM * TICKS_PER_REV / 60.0;
//        left.setVelocity(-ticks);
//        right.setVelocity(ticks);
//    }
//
//    public boolean atTarget() {
//        return Math.abs(getRPM(left) - targetRPM) < 75 &&
//                Math.abs(getRPM(right) - targetRPM) < 75;
//    }
//
//    private double getRPM(DcMotorEx m) {
//        return m.getVelocity() * 60 / TICKS_PER_REV;
//    }
//}
//
//
//package org.firstinspires.ftc.teamcode.subsystems;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//@Configurable
//public class Outtake {
//
//    private final DcMotorEx left, right;
//
//    public static double HIGH_RPM = 3400;
//    public static double LOW_RPM  = 2650;
//    public static double TICKS_PER_REV = 28;
//
//    public static double kP = 20, kI = 0, kD = 1.0;
//    public static double kF = (32767.0 / (6000 * TICKS_PER_REV)) * 60.0;
//
//    public double targetRPM = 0;
//    public boolean enabled = false;
//    private double lastTargetTicks = 0;
//
//    public Outtake(HardwareMap hw) {
//        left  = hw.get(DcMotorEx.class, "outtakeLeft");
//        right = hw.get(DcMotorEx.class, "outtakeRight");
//
//        left.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        right.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//
//        left.setVelocityPIDFCoefficients(kP, kI, kD, kF);
//        right.setVelocityPIDFCoefficients(kP, kI, kD, kF);
//    }
//
//    public void shootHigh() {
//        targetRPM = HIGH_RPM;
//        enabled = true;
//        updateVelocity();
//    }
//
//    public void shootLow() {
//        targetRPM = LOW_RPM;
//        enabled = true;
//        updateVelocity();
//    }
//
//    private void updateVelocity() {
//        double ticks = targetRPM * TICKS_PER_REV / 60.0;
//        if (Math.abs(ticks - lastTargetTicks) > 1) {
//            left.setVelocity(-ticks);
//            right.setVelocity(ticks);
//            lastTargetTicks = ticks;
//        }
//    }
//
//    public double getRPMLeft() {
//        return getRPM(left);
//    }
//
//    public double getRPMRight() {
//        return getRPM(right);
//    }
//
//    public double getTickLeft() {
//        return left.getVelocity();
//    }
//
//    public double getTickRight() {
//        return right.getVelocity();
//    }
//
//    public void stop() {
//        enabled = false;
//        left.setVelocity(0);
//        right.setVelocity(0);
//        lastTargetTicks = 0;
//    }
//
//    public void periodic() {
//        left.setVelocityPIDFCoefficients(kP, kI, kD, kF);
//        right.setVelocityPIDFCoefficients(kP, kI, kD, kF);
//        if (!enabled) {
//            if (Math.abs(lastTargetTicks) > 1) {
//                left.setVelocity(0);
//                right.setVelocity(0);
//                lastTargetTicks = 0;
//            }
//            return;
//        }
//        // PIDF controller maintains velocity automatically
//    }
//
//    public boolean atTarget() {
//        return Math.abs(getRPM(left) - targetRPM) < 60 &&
//                Math.abs(getRPM(right) - targetRPM) < 60;
//    }
//
//    private double getRPM(DcMotorEx m) {
//        return Math.abs(m.getVelocity() * 60 / TICKS_PER_REV);
//    }
//}


package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Outtake {

    private final DcMotorEx left, right;

    public static double HIGH_RPM = 4700;
    public static double LOW_RPM  = 3750;
    public static double TICKS_PER_REV = 28;

    // Manual PID constants (tune these)
    public static double kP = 0.066;
    public static double kI = 0.0001;
    public static double kD = 0.001;
    public static double kF = 0.12;  // Feedforward (open-loop power for target RPM)

    public double targetRPM = 0;
    public boolean enabled = false;

    // PID state variables
    private double leftErrorSum = 0, rightErrorSum = 0;
    private double leftLastError = 0, rightLastError = 0;
    private long lastTime = 0;

    public Outtake(HardwareMap hw) {
        left  = hw.get(DcMotorEx.class, "outtakeLeft");
        right = hw.get(DcMotorEx.class, "outtakeRight");

        left.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // No built-in PIDF - we do manual control
        stop();
    }

    public void shootHigh() {
        targetRPM = HIGH_RPM;
        enabled = true;
    }

    public void shootLow() {
        targetRPM = LOW_RPM;
        enabled = true;
    }

    public double getRPMLeft() {
        return getRPM(left);
    }

    public double getRPMRight() {
        return getRPM(right);
    }

    public double getTickLeft() {
        return left.getVelocity();
    }

    public double getTickRight() {
        return right.getVelocity();
    }

    public void setPower(double power) {
        right.setPower(power);
        left.setPower(-power);
    }

    public void stop() {
        enabled = false;
        left.setPower(0);
        right.setPower(0);
        resetPID();
    }

    public void periodic() {
        if (!enabled || targetRPM == 0) {
            left.setPower(0);
            right.setPower(0);
            return;
        }

        long currentTime = System.nanoTime();
        double dt = (currentTime - lastTime) / 1e9;
        if (dt < 0.001) return; // Skip if too fast
        lastTime = currentTime;

        // Left motor PID
        double leftRPM = getRPMLeft();
        double leftError = targetRPM - leftRPM;
        leftErrorSum += leftError * dt;
        double leftErrorDelta = leftError - leftLastError;

        double leftP = kP * leftError;
        double leftI = kI * leftErrorSum;
        double leftD = kD * (leftErrorDelta / dt);
        double leftPower = kF * (targetRPM / 6000.0) + leftP + leftI + leftD;

        // Right motor PID
        double rightRPM = getRPMRight();
        double rightError = targetRPM - rightRPM;
        rightErrorSum += rightError * dt;
        double rightErrorDelta = rightError - rightLastError;

        double rightP = kP * rightError;
        double rightI = kI * rightErrorSum;
        double rightD = kD * (rightErrorDelta / dt);
        double rightPower = kF * (targetRPM / 6000.0) + rightP + rightI + rightD;

        // Clamp outputs
        leftPower = Math.max(-1, Math.min(1, leftPower));
        rightPower = Math.max(-1, Math.min(1, rightPower));

        left.setPower(-leftPower);  // Negative for left direction
        right.setPower(rightPower);

        leftLastError = leftError;
        rightLastError = rightError;
    }

    public boolean atTarget() {
        double leftRPM = getRPMLeft();
        double rightRPM = getRPMRight();
        return Math.abs(leftRPM - targetRPM) < 60 &&
                Math.abs(rightRPM - targetRPM) < 60;
    }

    private double getRPM(DcMotorEx m) {
        return Math.abs(m.getVelocity() * 60 / TICKS_PER_REV);
    }

    private void resetPID() {
        leftErrorSum = 0;
        rightErrorSum = 0;
        leftLastError = 0;
        rightLastError = 0;
        lastTime = System.nanoTime();
    }
}
