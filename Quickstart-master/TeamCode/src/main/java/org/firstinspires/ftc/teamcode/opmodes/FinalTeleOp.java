package org.firstinspires.ftc.teamcode;

import static java.lang.Math.tan;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Teleop - Doremon", group = "TeleOp")
public class FinalTeleOp extends OpMode {

    /* ================= HARDWARE ================= */
    private DcMotorEx outtakeLeft, outtakeRight;
    private DcMotor intake, frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx turret;

    private Servo servoRight, servoLeft, servoGate;
    private Limelight3A limelight;

    /* ================= STATE ================= */
    private boolean isGateOpen = false;
    private boolean blueAlliance = true;

    private double lastTargetTicks = 0;
    private long stableStartTime = 0;
    private boolean rumbleTriggered = false;

    // DRIVETRAIN CONSTANTS
    private static final double DRIVETRAIN_SPEED = 0.95;
    private static final double STRAFE_MULTIPLIER = 1.1;
    private static final double ROTATION_MULTIPLIER = 0.6;

    // OUTTAKE CONSTANTS
    private static final double TICKS_PER_REV = 28;
    public static final double TARGET_RPM_HIGH = 4000;
    private static final double TARGET_RPM_LOW = 3000;
    double liveLowRPM = 1000;

    double P = 20.0;
    double F = 10.0;
    double rpmIncrement = 100;

    // TURRET CONSTANTS
    private static final double TURRET_TICKS_PER_REV = 8192.0;

    private static final double kP = 0.011;
    private static final double MAX_POWER = 0.8;
    private static final double TX_DEADBAND = 1.0;

    private static final double SEARCH_POWER = 0.24;
    private static final double SEARCH_ANGLE_DEG = 8.0;
    private static final double GEAR_RATIO = (double) 100 / 31;
    private static final double MANUAL_POWER = 0.35;
    private static final double CENTER_DEADBAND_DEG = 1.5;
    private static final double ANGLE_LIMIT_RIGHT = 7;
    private static final double ANGLE_LIMIT_LEFT = 6;

    // Manual Turret Control
    private boolean xWasPressed = false;
    private boolean manualCentering = false;


    // LIMELIGHT CONSTANTS
    final double MOUNT_ANGLE = 15;
    final double GOAL_HEIGHT = 39;
    final double LEMON_HEIGHT = 12.159379;

    // TURRET STATE MACHINE
    private enum State {
        SEARCH_LEFT,
        SEARCH_RIGHT,
        TRACKING,
        MANUAL
    }

    private State state = State.SEARCH_LEFT;

    // INIT
    @Override
    public void init() {

        outtakeLeft  = hardwareMap.get(DcMotorEx.class, "outtakeLeft");
        outtakeRight = hardwareMap.get(DcMotorEx.class, "outtakeRight");

        outtakeLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake = hardwareMap.get(DcMotor.class, "intake");

        servoRight = hardwareMap.get(Servo.class, "hoodServoRight");
        servoLeft = hardwareMap.get(Servo.class, "hoodServoLeft");
        servoGate = hardwareMap.get(Servo.class, "servoGate");

        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        backRight = hardwareMap.get(DcMotor.class, "br");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        setBrake(frontLeft, frontRight, backLeft, backRight,outtakeLeft, outtakeRight, intake);

        // PIDF
        double kP = 20, kI = 0, kD = 1.0;
        double kF = (32767.0 / (6000 * TICKS_PER_REV));

        outtakeLeft.setVelocityPIDFCoefficients(P, kI, kD, kF);
        outtakeRight.setVelocityPIDFCoefficients(P, kI, kD, kF);

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
    }

    @Override
    public void start() {
        limelight.start();

        servoRight.setPosition(1);
        servoLeft.setPosition(0);

        servoGate.setPosition(0);
    }

    // LOOP
    @Override
    public void loop() {
        controlOuttake();
        controlServo();
        controlIntake();
        controlDrivetrain();
        controlTurretTracking();
        updateTelemetry();
    }

    // TURRET
    private double getTurretAngleDeg() {
        return (turret.getCurrentPosition() / TURRET_TICKS_PER_REV) * 360.0 / GEAR_RATIO;
    }


    private void controlTurretTracking() {
        if (gamepad2.x ) {

            double turretAngle = getTurretAngleDeg();
            double power = 0;

            if (!xWasPressed) {
                manualCentering = true;
            }
            state = State.MANUAL;

            if (manualCentering) {
                if (Math.abs(turretAngle) > CENTER_DEADBAND_DEG) {
                    power = turretAngle > 0 ? -MANUAL_POWER : MANUAL_POWER;
                } else {
                    manualCentering = false;
                    power = 0;
                }
            } else {
                if (gamepad2.dpad_left) {
                    power = -MANUAL_POWER;
                } else if (gamepad2.dpad_right) {
                    power = MANUAL_POWER;
                } else {
                    power = 0; // hold position
                }
            }

            if (turretAngle <= -ANGLE_LIMIT_LEFT && power < 0) power = 0;
            if (turretAngle >= ANGLE_LIMIT_RIGHT && power > 0) power = 0;

            turret.setPower(power);

            telemetry.addData("State", "MANUAL");
            telemetry.addData("Turret Angle", turretAngle);
            telemetry.addData("Power", power);

            xWasPressed = true;
            return;
        }
        // X released → return to auto
        if (!gamepad2.x && xWasPressed) {
            xWasPressed = false;
            manualCentering = false;
        }

        LLResult result = limelight.getLatestResult();
        boolean hasTarget = result != null && result.isValid();

        double turretAngle = getTurretAngleDeg();
        double power = 0;

        if (hasTarget) {
            state = State.TRACKING;

            double tx = result.getTx();
            power = -kP * tx;

            if (Math.abs(tx) < TX_DEADBAND) power = 0;
            power = clamp(power, -MAX_POWER, MAX_POWER);

            telemetry.addData("State", "TRACKING");
            telemetry.addData("tx", tx);

        } else {

            if (state == State.TRACKING) {
                state = blueAlliance ? State.SEARCH_LEFT : State.SEARCH_RIGHT;
            }

            switch (state) {
                case SEARCH_LEFT:
                    if (turretAngle <= -SEARCH_ANGLE_DEG) {
                        state = State.SEARCH_RIGHT;
                    }
                    power = -SEARCH_POWER;
                    break;

                case SEARCH_RIGHT:
                    if (turretAngle >= SEARCH_ANGLE_DEG) {
                        state = State.SEARCH_LEFT;
                    }
                    power = SEARCH_POWER;
                    break;
            }


            telemetry.addData("State", state);
        }

        turret.setPower(power);

        telemetry.addData("Turret Angle", turretAngle);
        telemetry.addData("Power", power);
    }

    private double flywheelSpeed(double goalDist) {
        return 0;
    }

    // SERVOS
    private void controlServo() {
        if (gamepad2.bWasReleased()) {
            servoGate.setPosition(isGateOpen ? 0 : 0.61);
            isGateOpen = !isGateOpen;
        }

        if (gamepad2.dpadDownWasPressed()) {
            servoRight.setPosition(clamp(servoRight.getPosition() + 0.1, 0, 1));
            servoLeft.setPosition(clamp(servoLeft.getPosition() - 0.1, 0, 1));
        } else if (gamepad2.dpadUpWasPressed()) {
            servoRight.setPosition(clamp(servoRight.getPosition() - 0.1, 0, 1));
            servoLeft.setPosition(clamp(servoLeft.getPosition() + 0.1, 0, 1));

        }
        if (gamepad2.aWasPressed()) {
            servoRight.setPosition(0);
            servoLeft.setPosition(1);
        }
    }

    private double hoodAngle(double goalDist) {
        return 0;
    }

    // DRIVETRAIN
    private void controlDrivetrain() {
        double y2  = -gamepad2.left_stick_y;
        double x2  = gamepad2.left_stick_x * STRAFE_MULTIPLIER;
        double rx2 = gamepad2.right_stick_x * ROTATION_MULTIPLIER;

        double y1  = -gamepad1.left_stick_y;
        double x1  = gamepad1.left_stick_x * STRAFE_MULTIPLIER;
        double rx1 = gamepad1.right_stick_x * ROTATION_MULTIPLIER;

        double y, x, rx;

        if (Math.abs(y2) > 0.03 || Math.abs(x2) > 0.03 || Math.abs(rx2) > 0.03) {
            y = y2;
            x = x2;
            rx = rx2;
        } else {
            y = Math.abs(y1) > 0.03 ? y1 : 0;
            x = Math.abs(x1) > 0.03 ? x1 : 0;
            rx = Math.abs(rx1) > 0.03 ? rx1 : 0;
        }

        double fl = clamp((y + x + rx) * DRIVETRAIN_SPEED, -1, 1);
        double fr = clamp((y - x - rx) * DRIVETRAIN_SPEED, -1, 1);
        double bl = clamp((y - x + rx) * DRIVETRAIN_SPEED, -1, 1);
        double br = clamp((y + x - rx) * DRIVETRAIN_SPEED, -1, 1);

        setPowers(fl, fr, bl, br);
    }


    private void setPowers(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    // OUTTAKE
    private void controlOuttake() {
        double targetRPM =
                gamepad2.right_bumper ? TARGET_RPM_HIGH :
                        (gamepad2.right_trigger > 0.1 ? TARGET_RPM_LOW : 0);

        double targetTicks = targetRPM * TICKS_PER_REV / 60.0;

        if (Math.abs(targetTicks - lastTargetTicks) > 1e-3) {
            outtakeLeft.setVelocity(-targetTicks);
            outtakeRight.setVelocity(targetTicks);
            lastTargetTicks = targetTicks;
            stableStartTime = 0;
            rumbleTriggered = false;
        }

        if (targetRPM == 0) {
            outtakeLeft.setVelocity(0);
            outtakeRight.setVelocity(0);
            lastTargetTicks = 0;
            stableStartTime = 0;
            rumbleTriggered = false;
            return;
        }

        double leftRPM  = Math.abs(rpm(outtakeLeft));
        double rightRPM = Math.abs(rpm(outtakeRight));

        boolean inRange =
                Math.abs(leftRPM - targetRPM) < 60 &&
                        Math.abs(rightRPM - targetRPM) < 60;

        if (inRange) {
            if (stableStartTime == 0)
                stableStartTime = System.currentTimeMillis();

            if (!rumbleTriggered &&
                    System.currentTimeMillis() - stableStartTime > 50) {
                gamepad1.rumble(400);
                rumbleTriggered = true;
            }
        } else {
            stableStartTime = 0;
            rumbleTriggered = false;
        }
    }

    // INTAKE
    private void controlIntake() {
        if (gamepad2.left_bumper) {
            intake.setPower(-1);
        } else if (gamepad2.left_trigger > 0.1) {
            intake.setPower(1);
        } else {
            intake.setPower(0);
        }

        if (gamepad2.yWasPressed()) {
            intake.setPower(-.95);
        }
    }

    // LIMELIGHT DISTANCE
    private double getTargetDistance() {
        LLResult results = limelight.getLatestResult();

        if (results == null || !results.isValid()) {
            telemetry.addLine("No Tag Detected");
            return 0;
        }

        double ty = results.getTy();
        double angleRad = Math.toRadians(MOUNT_ANGLE + ty);
        double distance = (GOAL_HEIGHT - LEMON_HEIGHT) / Math.tan(angleRad);

        telemetry.addData("Tx", ty);
        telemetry.addData("Distance", distance);
        return distance;
    }

    // TELEMETRY
    private void updateTelemetry() {
        telemetry.addLine("----------------------- Outtake -----------------------");
        telemetry.addData("Target RPM",
                gamepad2.right_bumper ? TARGET_RPM_HIGH : (gamepad2.right_trigger > 0.1 ? TARGET_RPM_LOW : 0));

        telemetry.addData("Left RPM", rpm(outtakeLeft));
        telemetry.addData("Right RPM", rpm(outtakeRight));
        telemetry.addData("Left Ticks", outtakeLeft.getVelocity());
        telemetry.addData("Right Ticks", outtakeRight.getVelocity());

        telemetry.addLine("----------------------- Servos -----------------------");
        telemetry.addData("Gate Position", servoGate.getPosition());
        telemetry.addData("Right Servo", servoRight.getPosition());
        telemetry.addData("Left Servo", servoLeft.getPosition());

        telemetry.addLine("----------------------- Limelight -----------------------");
        telemetry.addData("Distance", getTargetDistance());

        telemetry.update();
    }

    // HELPERS
    private boolean turretCentered(double angleDeg) {
        return Math.abs(angleDeg) < CENTER_DEADBAND_DEG;
    }

    private double rpm(DcMotorEx motor) {
        return motor.getVelocity() * 60 / TICKS_PER_REV;
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    private void toggleAlliance() {
        if (gamepad1.aWasReleased()) {
            blueAlliance = !blueAlliance;
        }
    }

    private void setBrake(DcMotor... motors) {
        for (DcMotor m : motors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
}
