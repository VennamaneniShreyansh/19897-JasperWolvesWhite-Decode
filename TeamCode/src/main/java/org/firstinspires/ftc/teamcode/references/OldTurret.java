//////package org.firstinspires.ftc.teamcode.subsystems;
//////
//////import com.bylazar.configurables.annotations.Configurable;
//////import com.qualcomm.hardware.limelightvision.Limelight3A;
//////import com.qualcomm.hardware.limelightvision.LLResult;
//////import com.pedropathing.control.PIDFCoefficients;
//////import com.pedropathing.control.PIDFController;
//////import com.pedropathing.geometry.Pose;
//////import com.qualcomm.robotcore.hardware.DcMotor;
//////import com.qualcomm.robotcore.hardware.DcMotorEx;
//////import com.qualcomm.robotcore.hardware.HardwareMap;
//////import com.qualcomm.robotcore.util.ElapsedTime;
//////
//////@Configurable
//////public class Turret {
//////    public static double error = 0, power = 0, manualPower = 0;
//////    public static int offsetTicks = 0;
//////
////////    1. Motor CPR = 28 (GoBilda 5203 quadrature encoder)
////////    2. Planetary gear ratio = 13.7:1
////////    3. Spur gear ratio = driven ÷ driver = 100 ÷ 40 = 2.5:1
////////    4. Total gear ratio = 13.7 × 2.5 = 34.25:1
////////    5. Effective CPR at turret = 28 × 34.25 = 959 counts per turret revolution
////////    6. rpt = 2π ÷ 959 = 6.2832 ÷ 959 = 0.00655 rad/tick
//////    public static double rpt = 0.00436332313;
//////    private final double TICKS_PER_360 = 959;
//////
//////    private static final double TICKS_PER_360_TURRET_DEG = 1440;
//////
//////    public final DcMotorEx m;
////////    private PIDFController primaryPID, secondaryPID;
//////    public static double target = 0;
////////    public static double pidfSwitch = 5;
////////    public static double kp = 0.004, kf = 0.0001, kd = 0.000, sp = .03, sf = 0, sd = 0.0001;
////////
//////
//////    // New Stuff
//////    private double integralSum = 0;
//////    private double lastError = 0;
//////    private double lastReference = 0;
//////    private double previousFilterEstimate = 0;
//////    private double currentFilterEstimate = 0;
//////
//////    private final double a = 0.8; // low-pass filter constant
//////    private final double maxIntegralSum = 1000; // anti-windup limit
//////
//////    // PID constants; use new tuning values from Homeostasis model
//////    public static double Kp = 0.0083;
//////    public static double Ki = 0.0;
//////    public static double Kd = 0.0002;
//////
//////    private final ElapsedTime timer = new ElapsedTime();
//////
//////
//////    public static boolean on = true, manual = false;
//////
//////
//////
//////    public Turret(HardwareMap hardwareMap) {
//////        m = hardwareMap.get(DcMotorEx.class, "turret");
//////        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//////        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//////        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//////
//////    }
//////
//////    public void setTurretTarget(double ticks) {
//////        target = Math.max(-350, Math.min(345, ticks));
//////    }
//////
//////
//////    public double getTurretTarget() { return target; }
//////
//////    public double getTurret() {
//////        return m.getCurrentPosition() + offsetTicks;
//////    }
//////
//////    public double getTurretTicks() {
//////        return m.getCurrentPosition();
//////    }
//////    public double getTurretDegrees() {
//////        return (getTurretTicks() / TICKS_PER_360_TURRET_DEG) * 360.0;
//////    }
//////
//////public void periodic() {
//////    if (!on) {
//////        m.setPower(0);
//////        return;
//////    }
//////    if (manual) {
//////        m.setPower(manualPower);
//////        return;
//////    }
//////    double currentPosition = getTurret();
//////    double reference = getTurretTarget();
//////
//////    double error = reference - currentPosition;
//////
//////    double elapsedTime = timer.seconds();
//////    if (elapsedTime <= 0) elapsedTime = 1e-3;
//////    double errorChange = error - lastError;
//////
//////    currentFilterEstimate = (a * previousFilterEstimate) + ((1 - a) * errorChange);
//////    previousFilterEstimate = currentFilterEstimate;
//////
//////    double derivative = currentFilterEstimate / elapsedTime;
//////
//////    integralSum += error * elapsedTime;
//////    integralSum = Math.max(-maxIntegralSum, Math.min(maxIntegralSum, integralSum));
//////
//////    if (reference != lastReference) {
//////        integralSum = 0;
//////    }
//////
//////    double output = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
//////
//////    m.setPower(Math.max(-1, Math.min(1, output)));
//////
//////    lastError = error;
//////    lastReference = reference;
//////    timer.reset();
//////}
//////
//////
//////
//////    public void adjustOffsetRight(int ticks) {
//////        offsetTicks -= ticks;
//////        offsetTicks = Math.max(-200, Math.min(200, offsetTicks));
//////    }
//////    public void adjustOffsetLeft(int ticks) {
//////        offsetTicks += ticks;
//////        offsetTicks = Math.max(-50, Math.min(50, offsetTicks));
//////    }
//////    public void resetOffset() {
//////        offsetTicks = 0;
//////    }
//////    public double getOffsetTicks() { return offsetTicks; }
//////
//////    public void automatic() {
//////        manual = false;
//////    }
//////    public void on() { on = true; }
//////    public void off() { on = false; }
//////
//////    public double getYaw() { return normalizeAngle(getTurret() * rpt); }
//////
//////    public void setYaw(double radians) {
//////        radians = normalizeAngle(radians);
//////        setTurretTarget(radians / rpt);
//////    }
//////
//////    public void face(Pose targetPose, Pose robotPose) {
//////        double dx = targetPose.getX() - robotPose.getX();
//////        double dy = targetPose.getY() - robotPose.getY();
//////
//////        double targetAngle = Math.atan2(dy, dx);
//////        double turretYaw = normalizeAngle(targetAngle -
//////                robotPose.getHeading());
//////
//////        setYaw(turretYaw);
//////    }
//////
//////
//////    public void resetTurret() {
//////        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//////        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//////        setTurretTarget(0);
//////    }
//////
//////    public static double normalizeAngle(double angleRadians) {
//////        double angle = angleRadians % (Math.PI * 2D);
//////        if (angle <= -Math.PI) angle += Math.PI * 2D;
//////        if (angle > Math.PI) angle -= Math.PI * 2D;
//////        return angle;
//////    }
//////
//////    public void incrementTurretRight() {
//////        setTurretTarget(target -= 20);
//////    }
//////    public void incrementTurretLeft() {
//////        setTurretTarget(target += 20);
//////    }
//////
//////    public double angleDegreesToTicks(double angleDegrees) {
//////        double normalizedDegrees = normalizeAngleDegrees(angleDegrees);
//////        normalizedDegrees = Math.max(-270.0, Math.min(270.0, normalizedDegrees));
//////        double radians = Math.toRadians(normalizedDegrees);
//////        return radians / rpt;
//////    }
//////    private double normalizeAngleDegrees(double angleDegrees) {
//////        double angle = angleDegrees % 360.0;
//////        if (angle > 180.0) angle -= 360.0;
//////        if (angle <= -180.0) angle += 360.0;
//////        return angle;
//////    }
//////
//////
//////
//////    public double getError() { return error; }
//////}
//
//
//package org.firstinspires.ftc.teamcode.subsystems;
//
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
////    public static double Kp = 0.025;  // Increased for degrees scale
////    public static double Ki = 0.0;
////    public static double Kd = 0.0006; // Increased for degrees scale
//
//    public static double Kp = 0.08;
//    public static double Ki = 0.0;
//    public static double Kd = 0.00001;
//
//
//    private final ElapsedTime timer = new ElapsedTime();
//    public static boolean on = true, manual = false;
//
//    public Turret(HardwareMap hardwareMap) {
//        encoder = hardwareMap.get(AnalogInput.class, "turretEncoder");
//        m = hardwareMap.get(DcMotorEx.class, "turret");
//        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//    }
//
//
//
//    public static double ENCODER_ZERO_OFFSET_DEG = 1.215;
//    public static double lastAngle = 0;
//    public static double unwrappedAngle = 0;
//
//    public double getFinalVoltage() {
//        return encoder.getVoltage() - ENCODER_ZERO_OFFSET_DEG;
//    }
//
//    public double getTurretDegrees() {
//        double voltage = encoder.getVoltage();
//        double angle = (voltage / 3.3) * 360.0;
//        angle -= (ENCODER_ZERO_OFFSET_DEG/3.3)*360.0;
//        return angle;
//    }
//
//
//
//    public double getVoltage() {
//        return encoder.getVoltage();
//    }
//
//
//    public double getTurretTarget() { return targetDegrees; }
//
//    public void setTurretTarget(double degrees) {
//        targetDegrees = Math.max(-170, Math.min(135, degrees));
//    }
//
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
////        double currentPosition = getTurretDegrees();
////        double reference = getTurretTarget();
////
////        double error = reference - currentPosition;
//
//        double currentPosition = getTurretDegrees();
//        double reference = getTurretTarget();
//
//        double error = angleErrorDegrees(reference, currentPosition);
//
//
//
//        double elapsedTime = timer.seconds();
//        if (elapsedTime <= 0) elapsedTime = 1e-3;
//        double errorChange = error - lastError;
//
//        currentFilterEstimate = (a * previousFilterEstimate) + ((1 - a) * errorChange);
//        previousFilterEstimate = currentFilterEstimate;
//
//        double derivative = currentFilterEstimate / elapsedTime;
//
//        integralSum += error * elapsedTime;
//        integralSum = Math.max(-maxIntegralSum, Math.min(maxIntegralSum, integralSum));
//
//        if (reference != lastReference) {
//            integralSum = 0;
//        }
//
//        double output = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
//        m.setPower(Math.max(-1, Math.min(1, output)));
//
//        Turret.error = error;
//        lastError = error;
//        lastReference = reference;
//        timer.reset();
//    }
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
//    public double angleDegreesToTicks(double angleDegrees) {
//        return normalizeAngleDegrees(angleDegrees);
//    }
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
//
//
////////
////////
////////package org.firstinspires.ftc.teamcode.subsystems;
////////
////////import com.bylazar.configurables.annotations.Configurable;
////////import com.pedropathing.geometry.Pose;
////////import com.qualcomm.robotcore.hardware.DcMotor;
////////import com.qualcomm.robotcore.hardware.DcMotorEx;
////////import com.qualcomm.robotcore.hardware.HardwareMap;
////////import com.qualcomm.robotcore.util.ElapsedTime;
////////import com.acmerobotics.dashboard.config.Config;
////////
////////@Config
////////@Configurable
////////public class Turret {
////////
////////    // Public debug fields
////////    public static double error = 0, power = 0, manualPower = 0;
////////    public static int offsetTicks = 0;
////////
////////    // --- MECHANICAL CONSTANTS ---
////////    // 1. ELC encoder ticks per motor shaft revolution (fill this from ELC docs or measurement)
////////    // Example: if ELC gives 1440 counts per motor rev:
////////    public static double ELC_TICKS_PER_MOTOR_REV = 1440.0;  // TODO: set correctly
////////
////////    // 2. Gear ratio motor shaft : turret = 40 : 100 = 2.5:1
////////    public static double SPUR_RATIO = 100.0 / 40.0; // 2.5
////////
////////    // 3. If there is a planetary stage on the turret motor, include it:
////////    public static double PLANETARY_RATIO = 13.7; // if you're using 13.7:1 GoBilda
////////    // Total gear ratio motor encoder ticks : turret rev
////////    public static double TOTAL_RATIO = PLANETARY_RATIO * SPUR_RATIO; // 13.7 * 2.5 = 34.25
////////
////////    // 4. Effective ticks per turret revolution
////////    public static double TICKS_PER_TURRET_REV = ELC_TICKS_PER_MOTOR_REV * TOTAL_RATIO;
////////
////////    // 5. radians per tick (for getYaw/setYaw compatibility)
////////    public static double rpt = (2.0 * Math.PI) / TICKS_PER_TURRET_REV;
////////
////////    public final DcMotorEx m;
////////    public static double target = 0; // target in TICKS (turret ticks)
////////
////////    // PID internals
////////    private double integralSum = 0;
////////    private double lastError = 0;
////////    private double lastReference = 0;
////////    private double previousFilterEstimate = 0;
////////    private double currentFilterEstimate = 0;
////////
////////    private final double a = 0.8;     // low-pass filter constant
////////    private final double maxIntegralSum = 1000; // anti-windup limit (in ticks)
////////
////////    public static double Kp = 0.0083;
////////    public static double Ki = 0.0;
////////    public static double Kd = 0.0002;
////////
////////    private final ElapsedTime timer = new ElapsedTime();
////////
////////    public static boolean on = true, manual = false;
////////
////////    public Turret(HardwareMap hardwareMap) {
////////        // Motor "turret" has the ELC encoder plugged into its encoder port
////////        m = hardwareMap.get(DcMotorEx.class, "turret");
////////        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////////        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
////////        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
////////    }
////////
////////    // --- Targets ---
////////
////////    // Target in ticks, clamped to mechanical safe range
////////    public void setTurretTarget(double ticks) {
////////        // Clamp for safety; adjust these numbers to your real limits
////////        target = Math.max(-350, Math.min(345, ticks));
////////    }
////////
////////    public double getTurretTarget() { return target; }
////////
////////    // --- Position (ticks / degrees / yaw) ---
////////
////////    // Raw turret ticks (external encoder + offset)
////////    public double getTurret() {
////////        return m.getCurrentPosition() + offsetTicks;
////////    }
////////
////////    // Raw motor encoder ticks (without offset), if you want direct debug
////////    public double getTurretTicks() {
////////        return m.getCurrentPosition();
////////    }
////////
////////    // Turret angle in degrees (from ticks)
////////    public double getTurretDegrees() {
////////        return (getTurret() / TICKS_PER_TURRET_REV) * 360.0;
////////    }
////////
////////    // Radians, wrapped to [-π, π]
////////    public double getYaw() {
////////        return normalizeAngle(getTurret() * rpt);
////////    }
////////
////////    public void setYaw(double radians) {
////////        radians = normalizeAngle(radians);
////////        setTurretTarget(radians / rpt);
////////    }
////////
////////    // --- Main control loop ---
////////
////////    public void periodic() {
////////        if (!on) {
////////            m.setPower(0);
////////            return;
////////        }
////////        if (manual) {
////////            m.setPower(manualPower);
////////            return;
////////        }
////////
////////        double currentPosition = getTurret();      // ticks
////////        double reference = getTurretTarget();      // ticks
////////
////////        double e = reference - currentPosition;    // ticks of error
////////
////////        double elapsedTime = timer.seconds();
////////        if (elapsedTime <= 0) elapsedTime = 1e-3;
////////        double errorChange = e - lastError;
////////
////////        currentFilterEstimate = (a * previousFilterEstimate) + ((1 - a) * errorChange);
////////        previousFilterEstimate = currentFilterEstimate;
////////
////////        double derivative = currentFilterEstimate / elapsedTime;
////////
////////        integralSum += e * elapsedTime;
////////        integralSum = Math.max(-maxIntegralSum, Math.min(maxIntegralSum, integralSum));
////////
////////        if (reference != lastReference) {
////////            integralSum = 0;
////////        }
////////
////////        double output = (Kp * e) + (Ki * integralSum) + (Kd * derivative);
////////        output = Math.max(-1, Math.min(1, output));
////////
////////        m.setPower(output);
////////
////////        error = e;
////////        lastError = e;
////////        lastReference = reference;
////////        timer.reset();
////////    }
////////
////////    // --- Offset adjust (still supported) ---
////////
////////    public void adjustOffsetRight(int ticks) {
////////        offsetTicks -= ticks;
////////        offsetTicks = Math.max(-200, Math.min(200, offsetTicks));
////////    }
////////
////////    public void adjustOffsetLeft(int ticks) {
////////        offsetTicks += ticks;
////////        offsetTicks = Math.max(-50, Math.min(50, offsetTicks));
////////    }
////////
////////    public void resetOffset() {
////////        offsetTicks = 0;
////////    }
////////
////////    public double getOffsetTicks() { return offsetTicks; }
////////
////////    // --- Modes ---
////////
////////    public void automatic() {
////////        manual = false;
////////    }
////////
////////    public void on() { on = true; }
////////
////////    public void off() { on = false; }
////////
////////    // --- High-level aiming helpers (unchanged API) ---
////////
////////    public void face(Pose targetPose, Pose robotPose) {
////////        double dx = targetPose.getX() - robotPose.getX();
////////        double dy = targetPose.getY() - robotPose.getY();
////////
////////        double targetAngle = Math.atan2(dy, dx);
////////        double turretYaw = normalizeAngle(targetAngle - robotPose.getHeading());
////////
////////        setYaw(turretYaw);
////////    }
////////
////////    public void resetTurret() {
////////        // Reset external encoder through the motor's encoder port
////////        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////////        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
////////        offsetTicks = 0;
////////        setTurretTarget(0);
////////    }
////////
////////    public void incrementTurretRight() {
////////        setTurretTarget(target -= 20);
////////    }
////////
////////    public void incrementTurretLeft() {
////////        setTurretTarget(target += 20);
////////    }
////////
////////    // Degrees to ticks helper (same semantics as before)
////////    public double angleDegreesToTicks(double angleDegrees) {
////////        double normalizedDegrees = normalizeAngleDegrees(angleDegrees);
////////        normalizedDegrees = Math.max(-270.0, Math.min(270.0, normalizedDegrees));
////////        double radians = Math.toRadians(normalizedDegrees);
////////        return radians / rpt;
////////    }
////////
////////    // --- Utility ---
////////
////////    public static double normalizeAngle(double angleRadians) {
////////        double angle = angleRadians % (Math.PI * 2D);
////////        if (angle <= -Math.PI) angle += Math.PI * 2D;
////////        if (angle > Math.PI) angle -= Math.PI * 2D;
////////        return angle;
////////    }
////////
////////    private double normalizeAngleDegrees(double angleDegrees) {
////////        double angle = angleDegrees % 360.0;
////////        if (angle > 180.0) angle -= 360.0;
////////        if (angle <= -180.0) angle += 360.0;
////////        return angle;
////////    }
////////
////////    public double getError() { return error; }
////////}
////////
////////
////////package org.firstinspires.ftc.teamcode.subsystems;
////////
////////import com.acmerobotics.dashboard.config.Config;
////////import com.bylazar.configurables.annotations.Configurable;
////////import com.pedropathing.geometry.Pose;
////////import com.qualcomm.robotcore.hardware.DcMotor;
////////import com.qualcomm.robotcore.hardware.DcMotorEx;
////////import com.qualcomm.robotcore.hardware.DigitalChannel;
////////import com.qualcomm.robotcore.hardware.Encoder;
////////import com.qualcomm.robotcore.hardware.HardwareMap;
////////import com.qualcomm.robotcore.util.ElapsedTime;
////////
////////@Config
////////@Configurable
////////public class Turret {
////////    public static double error = 0, power = 0, manualPower = 0;
////////    public static int offsetTicks = 0;
////////
////////    // ELC Digital + Gear Ratios
////////    public static final double ELC_TICKS_PER_REV = 1440.0;  // ELC V2 digital CPR
////////    public static final double GEAR_RATIO = 100.0 / 40.0;   // 40T:100T = 2.5:1
////////    public static final double TICKS_PER_TURRET_REV = ELC_TICKS_PER_REV * GEAR_RATIO; // 3600 ticks/rev
////////    public static final double rpt = (2.0 * Math.PI) / TICKS_PER_TURRET_REV;
////////
////////    private final DigitalChannel encoder;      // ELC on digital ports
////////
////////    public final DcMotorEx m;           // Turret motor (no encoder needed)
////////    public static double target = 0;    // Target in TURRET TICKS
////////
////////    // PID state
////////    private double integralSum = 0;
////////    private double lastError = 0;
////////    private double lastReference = 0;
////////    private double previousFilterEstimate = 0;
////////    private double currentFilterEstimate = 0;
////////
////////    private final double a = 0.8;
////////    private final double maxIntegralSum = 900;  // tuned for ticks
////////
////////    public static double Kp = 0.0083;
////////    public static double Ki = 0.0;
////////    public static double Kd = 0.0002;
////////
////////    private final ElapsedTime timer = new ElapsedTime();
////////    public static boolean on = true, manual = false;
////////
////////    public Turret(HardwareMap hardwareMap) {
////////        encoder = hardwareMap.get(DigitalChannel.class, "turretEncoder");
////////        m = hardwareMap.get(DcMotorEx.class, "turret");
////////
////////        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
////////        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
////////    }
////////
////////    // === POSITION ===
////////    public double getTurretTicks() {
////////        return encoder.getCurrentPosition() + offsetTicks;
////////        encoder.
////////    }
////////
////////    public double getTurret() {
////////        return getTurretTicks();
////////    }
////////
////////    public double getTurretDegrees() {
////////        return (getTurretTicks() / TICKS_PER_TURRET_REV) * 360.0;
////////    }
////////
////////    public double getYaw() {
////////        return normalizeAngle(getTurretTicks() * rpt);
////////    }
////////
////////    // === TARGETS ===
////////    public void setTurretTarget(double ticks) {
////////        target = Math.max(-350, Math.min(345, ticks));
////////    }
////////
////////    public double getTurretTarget() {
////////        return target;
////////    }
////////
////////    public void setTurretTargetDegrees(double degrees) {
////////        setTurretTarget(degrees * TICKS_PER_TURRET_REV / 360.0);
////////    }
////////
////////    // === PID CONTROL ===
////////    public void periodic() {
////////        if (!on) {
////////            m.setPower(0);
////////            return;
////////        }
////////        if (manual) {
////////            m.setPower(manualPower);
////////            return;
////////        }
////////
////////        double currentPosition = getTurretTicks();
////////        double reference = getTurretTarget();
////////        double e = reference - currentPosition;
////////
////////        double elapsedTime = timer.seconds();
////////        if (elapsedTime <= 0) elapsedTime = 1e-3;
////////        double errorChange = e - lastError;
////////
////////        currentFilterEstimate = (a * previousFilterEstimate) + ((1 - a) * errorChange);
////////        previousFilterEstimate = currentFilterEstimate;
////////
////////        double derivative = currentFilterEstimate / elapsedTime;
////////
////////        integralSum += e * elapsedTime;
////////        integralSum = Math.max(-maxIntegralSum, Math.min(maxIntegralSum, integralSum));
////////
////////        if (reference != lastReference) {
////////            integralSum = 0;
////////        }
////////
////////        double output = (Kp * e) + (Ki * integralSum) + (Kd * derivative);
////////        m.setPower(Math.max(-1, Math.min(1, output)));
////////
////////        error = e;
////////        lastError = e;
////////        lastReference = reference;
////////        timer.reset();
////////    }
////////
////////    // === ALL YOUR EXISTING FUNCTIONS ===
////////    public void adjustOffsetRight(int ticks) {
////////        offsetTicks -= ticks;
////////        offsetTicks = Math.max(-200, Math.min(200, offsetTicks));
////////    }
////////
////////    public void adjustOffsetLeft(int ticks) {
////////        offsetTicks += ticks;
////////        offsetTicks = Math.max(-50, Math.min(50, offsetTicks));
////////    }
////////
////////    public void resetOffset() {
////////        offsetTicks = 0;
////////    }
////////
////////    public double getOffsetTicks() { return offsetTicks; }
////////
////////    public void automatic() { manual = false; }
////////    public void on() { on = true; }
////////    public void off() { on = false; }
////////
////////    public void setYaw(double radians) {
////////        radians = normalizeAngle(radians);
////////        setTurretTarget(radians / rpt);
////////    }
////////
////////    public void face(Pose targetPose, Pose robotPose) {
////////        double dx = targetPose.getX() - robotPose.getX();
////////        double dy = targetPose.getY() - robotPose.getY();
////////        double targetAngle = Math.atan2(dy, dx);
////////        double turretYaw = normalizeAngle(targetAngle - robotPose.getHeading());
////////        setYaw(turretYaw);
////////    }
////////
////////    public void resetTurret() {
////////        encoder.reset();
////////        offsetTicks = 0;
////////        setTurretTarget(0);
////////    }
////////
////////    public void incrementTurretRight() {
////////        setTurretTarget(target -= 20);
////////    }
////////
////////    public void incrementTurretLeft() {
////////        setTurretTarget(target += 20);
////////    }
////////
////////    public double angleDegreesToTicks(double angleDegrees) {
////////        double normalizedDegrees = normalizeAngleDegrees(angleDegrees);
////////        normalizedDegrees = Math.max(-270.0, Math.min(270.0, normalizedDegrees));
////////        double radians = Math.toRadians(normalizedDegrees);
////////        return radians / rpt;
////////    }
////////
////////    // === UTILITIES ===
////////    public static double normalizeAngle(double angleRadians) {
////////        double angle = angleRadians % (Math.PI * 2D);
////////        if (angle <= -Math.PI) angle += Math.PI * 2D;
////////        if (angle > Math.PI) angle -= Math.PI * 2D;
////////        return angle;
////////    }
////////
////////    private double normalizeAngleDegrees(double angleDegrees) {
////////        double angle = angleDegrees % 360.0;
////////        if (angle > 180.0) angle -= 360.0;
////////        if (angle <= -180.0) angle += 360.0;
////////        return angle;
////////    }
////////
////////    public double getError() { return error; }
////////    public double getVoltage() { return 0; } // Legacy, not used
////////}
////
////package org.firstinspires.ftc.teamcode.subsystems;
////
////import com.bylazar.configurables.annotations.Configurable;
////import com.qualcomm.hardware.limelightvision.Limelight3A;
////import com.qualcomm.hardware.limelightvision.LLResult;
////import com.pedropathing.control.PIDFCoefficients;
////import com.pedropathing.control.PIDFController;
////import com.pedropathing.geometry.Pose;
////import com.qualcomm.robotcore.hardware.DcMotor;
////import com.qualcomm.robotcore.hardware.DcMotorEx;
////import com.qualcomm.robotcore.hardware.HardwareMap;
////import com.qualcomm.robotcore.util.ElapsedTime;
////
////
////@Configurable
////public class Turret {
////    public static double error = 0, power = 0, manualPower = 0;
////    public static int offsetTicks = 0;
////
//////    1. Motor CPR = 28 (GoBilda 5203 quadrature encoder)
//////    2. Planetary gear ratio = 13.7:1
//////    3. Spur gear ratio = driven ÷ driver = 100 ÷ 40 = 2.5:1
//////    4. Total gear ratio = 13.7 × 2.5 = 34.25:1
//////    5. Effective CPR at turret = 28 × 34.25 = 959 counts per turret revolution
//////    6. rpt = 2π ÷ 959 = 6.2832 ÷ 959 = 0.00655 rad/tick
////public static double rpt = (2 * Math.PI) / (4000 * 2.5);
////
////    public final DcMotorEx m;
//////    private PIDFController primaryPID, secondaryPID;
////    public static double target = 0;
//////    public static double pidfSwitch = 5;
//////    public static double kp = 0.004, kf = 0.0001, kd = 0.000, sp = .03, sf = 0, sd = 0.0001;
//////
////
////    // New Stuff
////    private double integralSum = 0;
////    private double lastError = 0;
////    private double lastReference = 0;
////    private double previousFilterEstimate = 0;
////    private double currentFilterEstimate = 0;
////
////    private final double a = 0.8; // low-pass filter constant
////    private final double maxIntegralSum = 1000; // anti-windup limit
////
////    // PID constants; use new tuning values from Homeostasis model
////    public static double Kp = 0.0083;
////    public static double Ki = 0.0;
////    public static double Kd = 0.0002;
////
////    private final ElapsedTime timer = new ElapsedTime();
////
////
////    public static boolean on = true, manual = false;
////
////
////
////    public Turret(HardwareMap hardwareMap) {
////        m = hardwareMap.get(DcMotorEx.class, "turret");
////        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
////        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
////
//////        primaryPID = new PIDFController(new PIDFCoefficients(kp, 0, kd, kf));
//////        secondaryPID = new PIDFController(new PIDFCoefficients(sp, 0, sd, sf));
////    }
////
////    public void setTurretTarget(double ticks) {
////        target = Math.max(-3500, Math.min(3500, ticks));
////    }
////
////
////    public double getTurretTarget() { return target; }
////    public double getTurret() {
////        return m.getCurrentPosition();
////    }
////
////public void periodic() {
////    if (!on) {
////        m.setPower(0);
////        return;
////    }
////    if (manual) {
////        m.setPower(manualPower);
////        return;
////    }
////    double currentPosition = getTurret();
////    double reference = getTurretTarget();
////
////    double error = reference - currentPosition;
////
////    double elapsedTime = timer.seconds();
////    if (elapsedTime <= 0) elapsedTime = 1e-3;
////    double errorChange = error - lastError;
////
////    currentFilterEstimate = (a * previousFilterEstimate) + ((1 - a) * errorChange);
////    previousFilterEstimate = currentFilterEstimate;
////
////    double derivative = currentFilterEstimate / elapsedTime;
////
////    integralSum += error * elapsedTime;
////    integralSum = Math.max(-maxIntegralSum, Math.min(maxIntegralSum, integralSum));
////
////    if (reference != lastReference) {
////        integralSum = 0;
////    }
////
////    double output = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
////
////    m.setPower(Math.max(-1, Math.min(1, output)));
////
////    lastError = error;
////    lastReference = reference;
////    timer.reset();
////}
////
////
////
////    public void adjustOffsetRight(int ticks) {
////        offsetTicks -= ticks;
////        offsetTicks = Math.max(-200, Math.min(200, offsetTicks));
////    }
////    public void adjustOffsetLeft(int ticks) {
////        offsetTicks += ticks;
////        offsetTicks = Math.max(-50, Math.min(50, offsetTicks));
////    }
////    public void resetOffset() {
////        offsetTicks = 0;
////    }
////    public double getOffsetTicks() { return offsetTicks; }
////
////    public void automatic() {
////        manual = false;
////    }
////    public void on() { on = true; }
////    public void off() { on = false; }
////
////    public double getYaw() { return normalizeAngle(getTurret() * rpt); }
////
////    public void setYaw(double radians) {
////        radians = normalizeAngle(radians);
////        setTurretTarget(radians / rpt);
////    }
////
////    public void face(Pose targetPose, Pose robotPose) {
////        double dx = targetPose.getX() - robotPose.getX();
////        double dy = targetPose.getY() - robotPose.getY();
////
////        double targetAngle = Math.atan2(dy, dx);
////        double turretYaw = normalizeAngle(targetAngle -
////                robotPose.getHeading());
////
////        setYaw(turretYaw);
////    }
////
////
////    public void resetTurret() {
////        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
////        setTurretTarget(0);
////    }
////
////    public static double normalizeAngle(double angleRadians) {
////        double angle = angleRadians % (Math.PI * 2D);
////        if (angle <= -Math.PI) angle += Math.PI * 2D;
////        if (angle > Math.PI) angle -= Math.PI * 2D;
////        return angle;
////    }
////
////    public void incrementTurretRight() {
////        setTurretTarget(target -= 20);
////    }
////    public void incrementTurretLeft() {
////        setTurretTarget(target += 20);
////    }
////
////    public double angleDegreesToTicks(double angleDegrees) {
////        double normalizedDegrees = normalizeAngleDegrees(angleDegrees);
////        normalizedDegrees = Math.max(-270.0, Math.min(270.0, normalizedDegrees));
////        double radians = Math.toRadians(normalizedDegrees);
////        return radians / rpt;
////    }
////    private double normalizeAngleDegrees(double angleDegrees) {
////        double angle = angleDegrees % 360.0;
////        if (angle > 180.0) angle -= 360.0;
////        if (angle <= -180.0) angle += 360.0;
////        return angle;
////    }
////
////
////
////    public double getError() { return error; }
////}
//
////package org.firstinspires.ftc.teamcode.subsystems;
////
////import com.bylazar.configurables.annotations.Configurable;
////import com.qualcomm.hardware.limelightvision.Limelight3A;
////import com.qualcomm.hardware.limelightvision.LLResult;
////import com.pedropathing.control.PIDFCoefficients;
////import com.pedropathing.control.PIDFController;
////import com.pedropathing.geometry.Pose;
////import com.qualcomm.robotcore.hardware.DcMotor;
////import com.qualcomm.robotcore.hardware.DcMotorEx;
////import com.qualcomm.robotcore.hardware.HardwareMap;
////import com.qualcomm.robotcore.util.ElapsedTime;
////
////import org.firstinspires.ftc.robotcontroller.external.samples.RobotAutoDriveByGyro_Linear;
////
////@Configurable
////public class Turret {
////    public static double error = 0, power = 0, manualPower = 0;
////    public static int offsetTicks = 0;
////
//////    1. Motor CPR = 28 (GoBilda 5203 quadrature encoder)
//////    2. Planetary gear ratio = 13.7:1
//////    3. Spur gear ratio = driven ÷ driver = 100 ÷ 40 = 2.5:1
//////    4. Total gear ratio = 13.7 × 2.5 = 34.25:1
//////    5. Effective CPR at turret = 28 × 34.25 = 959 counts per turret revolution
//////    6. rpt = 2π ÷ 959 = 6.2832 ÷ 959 = 0.00655 rad/tick
////    public static double rpt = 0.00653;
////
////    public final DcMotorEx m;
//////    private PIDFController primaryPID, secondaryPID;
////    public static double target = 0;
//////    public static double pidfSwitch = 5;
//////    public static double kp = 0.004, kf = 0.0001, kd = 0.000, sp = .03, sf = 0, sd = 0.0001;
//////
////
////    // New Stuff
////    private double integralSum = 0;
////    private double lastError = 0;
////    private double lastReference = 0;
////    private double previousFilterEstimate = 0;
////    private double currentFilterEstimate = 0;
////
////    private final double a = 0.8; // low-pass filter constant
////    private final double maxIntegralSum = 1000; // anti-windup limit
////
////    // PID constants; use new tuning values from Homeostasis model
////    public static double Kp = 0.0083;
////    public static double Ki = 0.0;
////    public static double Kd = 0.0002;
////
////    private final ElapsedTime timer = new ElapsedTime();
////
////
////    public static boolean on = true, manual = false;
////
////
////
////    public Turret(HardwareMap hardwareMap) {
////        m = hardwareMap.get(DcMotorEx.class, "turret");
////        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
////        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
////
//////        primaryPID = new PIDFController(new PIDFCoefficients(kp, 0, kd, kf));
//////        secondaryPID = new PIDFController(new PIDFCoefficients(sp, 0, sd, sf));
////    }
////
////    public void setTurretTarget(double ticks) {
////        target = Math.max(-350, Math.min(345, ticks));
////    }
////
////
////    public double getTurretTarget() { return target; }
////    public double getTurret() {
////        return m.getCurrentPosition() + offsetTicks;
////    }
////
////public void periodic() {
////    if (!on) {
////        m.setPower(0);
////        return;
////    }
////    if (manual) {
////        m.setPower(manualPower);
////        return;
////    }
////    double currentPosition = getTurret();
////    double reference = getTurretTarget();
////
////    double error = reference - currentPosition;
////
////    double elapsedTime = timer.seconds();
////    if (elapsedTime <= 0) elapsedTime = 1e-3;
////    double errorChange = error - lastError;
////
////    currentFilterEstimate = (a * previousFilterEstimate) + ((1 - a) * errorChange);
////    previousFilterEstimate = currentFilterEstimate;
////
////    double derivative = currentFilterEstimate / elapsedTime;
////
////    integralSum += error * elapsedTime;
////    integralSum = Math.max(-maxIntegralSum, Math.min(maxIntegralSum, integralSum));
////
////    if (reference != lastReference) {
////        integralSum = 0;
////    }
////
////    double output = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
////
////    m.setPower(Math.max(-1, Math.min(1, output)));
////
////    lastError = error;
////    lastReference = reference;
////    timer.reset();
////}
////
////
////
////    public void adjustOffsetRight(int ticks) {
////        offsetTicks -= ticks;
////        offsetTicks = Math.max(-200, Math.min(200, offsetTicks));
////    }
////    public void adjustOffsetLeft(int ticks) {
////        offsetTicks += ticks;
////        offsetTicks = Math.max(-50, Math.min(50, offsetTicks));
////    }
////    public void resetOffset() {
////        offsetTicks = 0;
////    }
////    public double getOffsetTicks() { return offsetTicks; }
////
////    public void automatic() {
////        manual = false;
////    }
////    public void on() { on = true; }
////    public void off() { on = false; }
////
////    public double getYaw() { return normalizeAngle(getTurret() * rpt); }
////
////    public void setYaw(double radians) {
////        radians = normalizeAngle(radians);
////        setTurretTarget(radians / rpt);
////    }
////
////    public void face(Pose targetPose, Pose robotPose) {
////        double dx = targetPose.getX() - robotPose.getX();
////        double dy = targetPose.getY() - robotPose.getY();
////
////        double targetAngle = Math.atan2(dy, dx);
////        double turretYaw = normalizeAngle(targetAngle -
////                robotPose.getHeading());
////
////        setYaw(turretYaw);
////    }
////
////
////    public void resetTurret() {
////        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
////        setTurretTarget(0);
////    }
////
////    public static double normalizeAngle(double angleRadians) {
////        double angle = angleRadians % (Math.PI * 2D);
////        if (angle <= -Math.PI) angle += Math.PI * 2D;
////        if (angle > Math.PI) angle -= Math.PI * 2D;
////        return angle;
////    }
////
////    public void incrementTurretRight() {
////        setTurretTarget(target -= 20);
////    }
////    public void incrementTurretLeft() {
////        setTurretTarget(target += 20);
////    }
////
////    public double angleDegreesToTicks(double angleDegrees) {
////        double normalizedDegrees = normalizeAngleDegrees(angleDegrees);
////        normalizedDegrees = Math.max(-270.0, Math.min(270.0, normalizedDegrees));
////        double radians = Math.toRadians(normalizedDegrees);
////        return radians / rpt;
////    }
////    private double normalizeAngleDegrees(double angleDegrees) {
////        double angle = angleDegrees % 360.0;
////        if (angle > 180.0) angle -= 360.0;
////        if (angle <= -180.0) angle += 360.0;
////        return angle;
////    }
////
////
////
////    public double getError() { return error; }
////
////}