package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Dore-Le-mon Teleop", group = "TeleOp")
public class TeleOpSingleLL extends OpMode {

    private DcMotorEx outtakeLeft, outtakeRight;
    private DcMotor intake, frontLeft, frontRight, backLeft, backRight;
    private Servo servo;
    private static final double DRIVETRAIN_SPEED = 0.95;
    private static final double STRAFE_MULTIPLIER = 1.1;
    private static final double ROTATION_MULTIPLIER = 0.6;

    private static final double TICKS_PER_REV = 28;
    private static final double TARGET_RPM_HIGH = 3400;
    private static final double TARGET_RPM_LOW = 2650;

    private double lastTargetTicks = 0;
    private long stableStartTime = 0;
    private boolean rumbleTriggered = false;
    private boolean blueAlliance = true;
    private Limelight3A limelight;
    private double lastError = 0;

    @Override
    public void init() {
        outtakeLeft = hardwareMap.get(DcMotorEx.class, "outtakeLeft");
        outtakeRight = hardwareMap.get(DcMotorEx.class, "outtakeRight");
        intake = hardwareMap.get(DcMotor.class,    "intake");
        servo = hardwareMap.get(Servo.class, "servo");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        setBrake(frontLeft, frontRight, backLeft, backRight, outtakeLeft, outtakeRight, intake);

        // PIDF
        double kP = 20, kI = 0, kD = 1.0;
        double kF = (32767.0 / (6000 * TICKS_PER_REV)) * 60.0;

        outtakeLeft.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        outtakeRight.setVelocityPIDFCoefficients(kP, kI, kD, kF);

        // Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_right) {
            servo.setPosition(0);
        } else if (gamepad1.dpad_left){
            servo.setPosition(0.15);
        }

        toggleAlliance();
        controlOuttake();
        controlIntake();
        controlDrivetrain();
        updateTelemetry();
    }

    private void updateTelemetry() {
        if (blueAlliance) telemetry.addLine("Blue Alliance");
        else telemetry.addLine("Red Alliance");
        telemetry.addLine("----------------------- Outtake -----------------------");
        telemetry.addData("Target RPM", gamepad1.right_bumper ? TARGET_RPM_HIGH :
                (gamepad1.right_trigger > 0.1 ? TARGET_RPM_LOW : 0));
        telemetry.addData("Left RPM",  rpm(outtakeLeft));
        telemetry.addData("Right RPM", rpm(outtakeRight));
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

        if (gamepad1.b) {
            rx = getLemonStuff();
        }

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
        double targetRPM = gamepad1.right_bumper ? TARGET_RPM_HIGH : (gamepad1.right_trigger > 0.1 ? TARGET_RPM_LOW : 0);

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
        if (gamepad1.left_bumper) {
            intake.setPower(0.95);
        } else if (gamepad1.left_trigger > 0.1) {
            intake.setPower(-0.95);
        } else {
            intake.setPower(0);
        }
    }

    private double getLemonStuff() {
        LLResult results = limelight.getLatestResult();

        if (results == null || !results.isValid()) {
            telemetry.addLine("No Tag Detected");
            return 0;
        }

        double yaw = results.getTx() + ((blueAlliance) ? -4 : 4);  // offset
        double derivative = yaw - lastError;
        lastError = yaw;

        double kP = 0.03;
        double kD = 0.001;

        double output = kP * yaw + kD * derivative;
        output = clamp(output);

        telemetry.addData("Tx", yaw);
        telemetry.addData("TurnPower", output);

        return output;
    }

    private double clamp(double v) {
        return Math.max(-0.5, Math.min(0.5, v));
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
