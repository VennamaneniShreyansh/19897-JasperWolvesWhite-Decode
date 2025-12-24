package org.firstinspires.ftc.teamcode.references;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp(name = "Limelight AutoAlign TeleOp")
@Disabled
public class MoreStuffWithLimelight extends OpMode {

    // Drivetrain motors
    private DcMotor fl, fr, bl, br;

    // Limelight camera
    private Limelight3A limelight;
    private IMU imu;

    // PID constants (tune if needed)
    private double kP = 0.03;
    private double kD = 0.001;

    private double lastError = 0;

    @Override
    public void init() {
        // Map drivetrain motors
        fl = hardwareMap.get(DcMotor.class, "frontLeft");
        fr = hardwareMap.get(DcMotor.class, "frontRight");
        bl = hardwareMap.get(DcMotor.class, "backLeft");
        br = hardwareMap.get(DcMotor.class, "backRight");

        // Motor directions (adjust if reversed)
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        // Zero power behavior for control
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Camera
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        imu.initialize(new IMU.Parameters(orientation));
        telemetry.addLine("Initialized. Press PLAY.");
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {

        // Normal driver inputs
        double y  = -gamepad1.left_stick_y;  // forward/back
        double x  = gamepad1.left_stick_x;   // strafe
        double rx = gamepad1.right_stick_x;  // turn

        // If B is held → auto-align to AprilTag using yaw
        if (gamepad1.b) {
            rx = getTagYawCorrection();   // Override rotation power
        }

        // Mecanum drive math
        double flPower = y + x + rx;
        double frPower = y - x - rx;
        double blPower = y - x + rx;
        double brPower = y + x - rx;

        // Normalize if any power >1
        double max = Math.max(1.0,
                Math.max(Math.abs(flPower),
                        Math.max(Math.abs(frPower),
                                Math.max(Math.abs(blPower),
                                        Math.abs(brPower)))));

        fl.setPower(flPower / max);
        fr.setPower(frPower / max);
        bl.setPower(blPower / max);
        br.setPower(brPower / max);

        telemetry.addData("Aligning?", gamepad1.b);
        telemetry.update();
    }

    private double getTagYawCorrection() {
        LLResult results = limelight.getLatestResult();
        boolean hasTarget = results != null && results.isValid();

        if (!hasTarget) {
            telemetry.addLine("No Tag Detected");
            return 0;
        }

        double yaw = results.getTx();  // positive = tag is right, negative = left

        // PD control
        double derivative = yaw - lastError;
        lastError = yaw;

        double output = (kP * yaw) + (kD * derivative);

        // Limit turn power
        output = Math.max(-0.5, Math.min(0.5, output));

        telemetry.addData("Yaw", yaw);
        telemetry.addData("TurnPower", output);

        return output;
    }
}
