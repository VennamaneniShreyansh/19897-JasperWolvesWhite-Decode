package org.firstinspires.ftc.teamcode;

import static java.lang.Math.tan;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Teleop New", group = "TeleOp")
public class TeleopNew extends OpMode {

    private DcMotorEx outtakeLeft, outtakeRight;
    private DcMotor intake, frontLeft, frontRight, backLeft, backRight;
    private Servo servoRight, servoLeft, servoGate;
    private boolean isGateOpen = false;
    private static final double DRIVETRAIN_SPEED = 0.95;
    private static final double STRAFE_MULTIPLIER = 1.1;
    private static final double ROTATION_MULTIPLIER = 0.6;

    private static final double TICKS_PER_REV = 28;
    private static final double TURRET_TICKS_PER_REV = 8192.0;

    private static double TARGET_RPM_HIGH = 5400;
    private static final double TARGET_RPM_LOW = 3000;


    private double lastTargetTicks = 0;
    private long stableStartTime = 0;
    private boolean rumbleTriggered = false;
    private boolean blueAlliance = true;
    private Limelight3A limelight;
    final double MOUNT_ANGLE = 1;
    final double GOAL_HEIGHT = 10;
    final double LEMON_HEIGHT = 5;
//    private double lastError = 0;
    double P = 70.0;
    double F = 10.0;

    private DcMotorEx turret;
    private enum State {
        SEARCH_LEFT,
        SEARCH_RIGHT,
        TRACKING
    }
    private State state = State.SEARCH_LEFT;

    private static final double kP = 0.011;
    private static final double MAX_POWER = 0.8;
    private static final double TX_DEADBAND = 1.0;

    private static final double SEARCH_POWER = 0.2;
    private static final double SEARCH_ANGLE_DEG = 8.0;
    private static final double GEAR_RATIO = (double) 100 /31;

    @Override
    public void init() {
        outtakeLeft = hardwareMap.get(DcMotorEx.class, "outtakeLeft");
        outtakeRight = hardwareMap.get(DcMotorEx.class, "outtakeRight");
        outtakeRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake = hardwareMap.get(DcMotor.class,    "intake");

        servoRight = hardwareMap.get(Servo.class, "hoodServoRight");
        servoLeft = hardwareMap.get(Servo.class, "hoodServoLeft");
        servoGate = hardwareMap.get(Servo.class, "servoGate");

        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        backRight = hardwareMap.get(DcMotor.class, "br");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        setBrake(frontLeft, frontRight, backLeft, backRight, outtakeLeft, outtakeRight, intake);

        // PIDF
        double kP = 20, kI = 0, kD = 1.0;
        double kF = (32767.0 / (6000 * TICKS_PER_REV)) * 60.0;

        outtakeLeft.setVelocityPIDFCoefficients(P, kI, kD, kF);
        outtakeRight.setVelocityPIDFCoefficients(P, kI, kD, F);

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

    @Override
    public void loop() {
        controlOuttake();
        controlServo();
        controlIntake();
        controlDrivetrain();
        controlTurretTracking();
        updateTelemetry();
    }
    private double getTurretAngleDeg() {
        return (turret.getCurrentPosition() / TURRET_TICKS_PER_REV) * 360.0;
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
    private void controlTurretTracking() {
        LLResult result = limelight.getLatestResult();
        boolean hasTarget = result != null && result.isValid();

        double turretAngle = getTurretAngleDeg();
        double power = 0;

        // Tracking
        if (hasTarget) {
            state = State.TRACKING;

            double tx = result.getTx();
            power = -kP * tx;

            if (Math.abs(tx) < TX_DEADBAND) {
                power = 0;
            }

            power = clamp(power, -MAX_POWER, MAX_POWER);

            telemetry.addData("State", "TRACKING");
            telemetry.addData("tx", tx);
        }

        // Search
        else {
            if (state == State.TRACKING) {
                // if lose target, search left
                state = State.SEARCH_LEFT;
            }
            switch (state) {

                case SEARCH_LEFT:
                    power = -SEARCH_POWER;
                    if (turretAngle <= -SEARCH_ANGLE_DEG) {
                        state = State.SEARCH_RIGHT;
                    }
                    break;

                case SEARCH_RIGHT:
                    power = SEARCH_POWER;
                    if (turretAngle >= SEARCH_ANGLE_DEG) {
                        state = State.SEARCH_LEFT;
                    }
                    break;
            }

            telemetry.addData("State", state);
        }

        turret.setPower(power);

        telemetry.addData("Turret Angle", turretAngle);
        telemetry.addData("Power", power);
//        telemetry.update();
    }

    private void controlServo() {
        if (gamepad2.bWasReleased()) {
            if (isGateOpen) {
                servoGate.setPosition(0);
            } else {
                servoGate.setPosition(.61);
            }
            isGateOpen = !isGateOpen;
        }

        if (gamepad2.yWasReleased()) {
            servoRight.setPosition(servoRight.getPosition()+.1);
            servoLeft.setPosition(servoLeft.getPosition()-.1);
        } else if (gamepad2.aWasReleased()) {
            servoRight.setPosition(servoRight.getPosition()-.1);
            servoLeft.setPosition(servoLeft.getPosition()+.1);
        }
    }

    private void updateTelemetry() {
        telemetry.addLine("----------------------- Outtake -----------------------");
        telemetry.addData("Target RPM", gamepad1.right_bumper ? TARGET_RPM_HIGH :
                (gamepad1.right_trigger > 0.1 ? TARGET_RPM_LOW : 0));
        telemetry.addData("Left RPM",  rpm(outtakeLeft));
        telemetry.addData("Right RPM", rpm(outtakeRight));
        telemetry.addLine("----------------------- Servos -----------------------");
        telemetry.addData("Gate Position", servoGate.getPosition());
        telemetry.addData("Right Servo",  servoRight.getPosition());
        telemetry.addData("Left Servo", servoLeft.getPosition());
        telemetry.update();
    }

    // Converts motor velocity to RPM
    private double rpm(DcMotorEx motor) {
        return motor.getVelocity() * 60 / TICKS_PER_REV;
    }

    private void controlDrivetrain() {
        double y  = -gamepad1.left_stick_y;
        double x  = gamepad1.left_stick_x  * STRAFE_MULTIPLIER;
        double rx = gamepad1.right_stick_x * ROTATION_MULTIPLIER;

        if (Math.abs(y) < 0.03) y = 0;
        if (Math.abs(x) < 0.03) x = 0;
        if (Math.abs(rx) < 0.03) rx = 0;

        double fl = (y + x + rx) * DRIVETRAIN_SPEED;
        double fr = (y - x - rx) * DRIVETRAIN_SPEED;
        double bl = (y - x + rx) * DRIVETRAIN_SPEED;
        double br = (y + x - rx) * DRIVETRAIN_SPEED;

        setPowers(fl, fr, bl, br);
    }

    private void setPowers(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    private void controlOuttake() {
        double targetRPM = gamepad2.right_bumper ? TARGET_RPM_HIGH : (gamepad2.right_trigger > 0.1 ? TARGET_RPM_LOW : 0);

        double targetTicks = targetRPM * TICKS_PER_REV / 60.0;

        if (targetTicks != lastTargetTicks) {
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

        boolean inRange = Math.abs(leftRPM - targetRPM) < 60 &&
                Math.abs(rightRPM - targetRPM) < 60;

        if (inRange) {
            if (stableStartTime == 0)
                stableStartTime = System.currentTimeMillis();

            if (!rumbleTriggered && System.currentTimeMillis() - stableStartTime > 50) {
                gamepad1.rumble(400);
                rumbleTriggered = true;
            }
        } else {
            stableStartTime = 0;
            rumbleTriggered = false;
        }
    }

    private void controlIntake() {
        if (gamepad2.left_bumper) {
            intake.setPower(-0.95);
        } else if (gamepad2.left_trigger > 0.1) {
            intake.setPower(0.95);
        } else {
            intake.setPower(0);
        }
    }

    private double getTargetDistance() {
        LLResult results = limelight.getLatestResult();

        if (results == null || !results.isValid()) {
            telemetry.addLine("No Tag Detected");
            return 0;
        }

        double ty = results.getTy();
        double distance = (GOAL_HEIGHT-LEMON_HEIGHT) / tan(MOUNT_ANGLE+ty);
        telemetry.addData("Tx", ty);
        telemetry.addData("Distance", distance);

        return distance;
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
