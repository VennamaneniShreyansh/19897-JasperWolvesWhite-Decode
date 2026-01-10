package org.firstinspires.ftc.teamcode.references;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Simple Outtake OpMode", group = "TeleOp")
@Disabled
public class SimpleOuttakeOpMode extends OpMode {

    private DcMotor outtakeLeft, outtakeRight;
    private DcMotor intake;

    @Override
    public void init() {
        outtakeLeft = hardwareMap.get(DcMotor.class, "outtakeLeft");
        outtakeRight = hardwareMap.get(DcMotor.class, "outtakeRight");

//        outtakeLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        outtakeRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        intake = hardwareMap.get(DcMotor.class, "intake");
    }

    @Override
    public void loop() {
        // Right bumper = MAX POWER OUTTAKE
        if (gamepad1.right_bumper) {
            outtakeLeft.setPower(-1.0);   // Full power, reversed direction
            outtakeRight.setPower(1.0);   // Full power
            intake.setPower(-1);
            telemetry.addLine("OUTTAKE: FULL POWER ON");
        } else {
            outtakeLeft.setPower(0);
            outtakeRight.setPower(0);
            telemetry.addLine("OUTTAKE: OFF");
        }


        // Telemetry
//        telemetry.addData("Left RPM", outtakeLeft.getVelocity() * 60 / 28.0);
//        telemetry.addData("Right RPM", outtakeRight.getVelocity() * 60 / 28.0);
    }
}
