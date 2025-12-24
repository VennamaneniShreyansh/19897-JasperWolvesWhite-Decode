package org.firstinspires.ftc.teamcode.references;
 import com.pedropathing.follower.Follower;
 import com.qualcomm.robotcore.eventloop.opmode.Disabled;
 import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "AtThisPointIDK", group = "TeleOp")
@Disabled
public class PedroTeleop extends OpMode {
     private Follower follower;
    private DcMotorEx outtakeLeft, outtakeRight;
    private DcMotor intake;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    double targetRPM = 2567;
    double targetRPMLow = 2067;
    private double ticksPerRev;
    private boolean rumbleTriggered = false;

    @Override
    public void init() {
        outtakeLeft = hardwareMap.get(DcMotorEx.class, "outtakeLeft");
        outtakeRight = hardwareMap.get(DcMotorEx.class, "outtakeRight");
        intake = hardwareMap.get(DcMotor.class, "intake");

        outtakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        outtakeLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void start() {
        ticksPerRev = 28;
    }

    @Override
    public void loop() {
        if (gamepad2.right_bumper) {
            double targetTicksPerSec = targetRPM * ticksPerRev / 60.0;
            outtakeLeft.setVelocity(-targetTicksPerSec);
            outtakeRight.setVelocity(targetTicksPerSec);
        } else if (gamepad2.right_trigger > 0) {
            double targetTicksPerSec = targetRPMLow * ticksPerRev / 60.0;
            outtakeLeft.setVelocity(-targetTicksPerSec);
            outtakeRight.setVelocity(targetTicksPerSec);
        } else {
            outtakeLeft.setVelocity(0);
            outtakeRight.setVelocity(0);
        }

        double leftRPM = Math.abs(outtakeLeft.getVelocity() * 60 / ticksPerRev);
        double rightRPM = Math.abs(outtakeRight.getVelocity() * 60 / ticksPerRev);
        boolean ready = Math.abs(leftRPM - targetRPM) < 100 && Math.abs(rightRPM - targetRPM) < 100;
        if (ready && !rumbleTriggered) {
            gamepad2.rumble(500);
            rumbleTriggered = true;
        } else if (!ready) {
            rumbleTriggered = false;
        }

        if (gamepad2.left_bumper) {
            intake.setPower(.95);
        } else if (gamepad2.left_trigger > 0) {
            intake.setPower(-.95);
        } else {
            intake.setPower(0);
        }

        if (gamepad1.dpadUpWasReleased()) {
            targetRPM = targetRPM + 100;
        } else if (gamepad1.dpadDownWasReleased()) {
            targetRPM = targetRPM - 100;
        }

        controlDrivetrain();
        updateTelemetry();
    }

    public void updateTelemetry() {
        telemetry.addData("Current target RPM ", targetRPM);
        telemetry.addData("Left RPM ", outtakeLeft.getVelocity() * 60 / ticksPerRev);
        telemetry.addData("Right RPM ", outtakeRight.getVelocity() * 60 / ticksPerRev);
    }

    private void controlDrivetrain() {
        if (gamepad1.dpad_up) SetDrivetrainMotorPowers(.9, .9, .9, .9);
        else if (gamepad1.dpad_down) SetDrivetrainMotorPowers(-.9, -.9, -.9, -.9);
        else if (gamepad1.dpad_left) SetDrivetrainMotorPowers(-.9, .9, .9, -.9);
        else if (gamepad1.dpad_right) SetDrivetrainMotorPowers(.9, -.9, -.9, .9);
        else {
            // Default joystick control
            double y = -gamepad1.left_stick_y;  // Forward/backward
            double x = gamepad1.left_stick_x * 1.1;  // Strafe
            double rx = gamepad1.right_stick_x * .6;  // Rotation


            // different drive train
            double frontLeftPower = (y + x + rx) * .9;
            double frontRightPower = (y - x - rx) * .9;
            double backLeftPower = (y - x + rx) * .9;
            double backRightPower = (y + x - rx) * .9;

            SetDrivetrainMotorPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
        }
    }

    private void SetDrivetrainMotorPowers(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }
}