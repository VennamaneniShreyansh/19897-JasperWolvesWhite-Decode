package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import java.util.List;

@Configurable
@Config
public class Outtake {

    private final MotorEx left, right;
    private final List<LynxModule> allHubs;
    // target RPMs
    public static double HIGH_RPM = 4450;
    public static double LOW_RPM  = 3700;
    public static double HIGH_HIGH_RPM = 4450;
    public static double TICKS_PER_REV = 28.0;

    public static double kP = 0.00005;
    public static double kI = 0.0;
    public static double kD = 0.00006;

    // Feedforward (kS + kV + kA) are unitless motor model gains
    public static double kS = .00007;
    public static double kV = 0.0005;
    public static double kA = 0.0;   // start at 0 unless you need accel comp


    private static final double RPM_FILTER_ALPHA = 0.67;

    private double leftFilteredRPM = 0.0;
    private double rightFilteredRPM = 0.0;
    private boolean rpmFilterInitialized = false;


    public double targetRPM = 0;
    private boolean enabled = false;
    private long stableStartTime = 0;

    private double lowPass(double previous, double current) {
        return RPM_FILTER_ALPHA * previous + (1.0 - RPM_FILTER_ALPHA) * current;
    }


    public Outtake(HardwareMap hw) {
        allHubs = hw.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        left  = new MotorEx(hw, "outtakeLeft");
        right = new MotorEx(hw, "outtakeRight");

        left.setInverted(true);
        right.setInverted(false);

        // motor run mode: velocity controlled
        left.setRunMode(MotorEx.RunMode.VelocityControl);
        right.setRunMode(MotorEx.RunMode.VelocityControl);

        // configure coeffs once
        updateVeloPIDF();

        stop();
    }

    private void updateVeloPIDF() {
        // configure PID
        left.setVeloCoefficients(kP, kI, kD);
        right.setVeloCoefficients(kP, kI, kD);

        // configure feedforward
        if (kA == 0.0) {
            left.setFeedforwardCoefficients(kS, kV);
            right.setFeedforwardCoefficients(kS, kV);
        } else {
            left.setFeedforwardCoefficients(kS, kV, kA);
            right.setFeedforwardCoefficients(kS, kV, kA);
        }
    }

    public void shootHigh() { setTargetRPM(HIGH_RPM); }
    public void shootLow()  { setTargetRPM(LOW_RPM); }
    public void shootDoubleHigh() {setTargetRPM(HIGH_HIGH_RPM);}

    public void stop() {
        enabled = false;
        targetRPM = 0;
        stableStartTime = 0;

        rpmFilterInitialized = false;
        leftFilteredRPM = 0.0;
        rightFilteredRPM = 0.0;

        left.setRunMode(MotorEx.RunMode.RawPower);
        right.setRunMode(MotorEx.RunMode.RawPower);
        left.set(0);
        right.set(0);
    }


    public void setTargetRPM(double rpm) {
        targetRPM = rpm;
        enabled = rpm != 0;
        stableStartTime = 0;

        if (!enabled) {
            stop();
            return;
        }
        // ensure velocity mode is active
        left.setRunMode(MotorEx.RunMode.VelocityControl);
        right.setRunMode(MotorEx.RunMode.VelocityControl);

        double ticksPerSec = rpm * TICKS_PER_REV / 60.0;

        left.set(ticksPerSec);
        right.set(ticksPerSec);
    }

    public void periodic() {
        if (!enabled || targetRPM == 0) {
            // keep motors stopped if not enabled
            left.setRunMode(MotorEx.RunMode.RawPower);
            right.setRunMode(MotorEx.RunMode.RawPower);
            left.set(0);
            right.set(0);
            return;
        }

        updateVeloPIDF();

        double ticksPerSec = targetRPM * TICKS_PER_REV / 60.0;
        left.set(ticksPerSec);
        right.set(ticksPerSec);
    }


    public double getRPMLeft() {
        double rawRPM =
                Math.abs(left.getVelocity() * 60.0 / TICKS_PER_REV);

        if (!rpmFilterInitialized) {
            leftFilteredRPM = rawRPM;
            rpmFilterInitialized = true;
            return rawRPM;
        }

        leftFilteredRPM = lowPass(leftFilteredRPM, rawRPM);
        return leftFilteredRPM;
    }


    public double getRPMRight() {
        double rawRPM =
                Math.abs(right.getVelocity() * 60.0 / TICKS_PER_REV);

        if (!rpmFilterInitialized) {
            rightFilteredRPM = rawRPM;
            rpmFilterInitialized = true;
            return rawRPM;
        }

        rightFilteredRPM = lowPass(rightFilteredRPM, rawRPM);
        return rightFilteredRPM;
    }


    public double getTickLeft()  { return left.getVelocity(); }
    public double getTickRight() { return right.getVelocity(); }
    public boolean isEnabled()   { return enabled; }
}
