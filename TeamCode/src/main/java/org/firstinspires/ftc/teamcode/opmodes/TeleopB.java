package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.helper.Alliance;

@TeleOp(name = "Main TeleOp Blue", group = "TeleOp")
public class TeleopB extends OpMode {

    private Robot robot;
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;
    private boolean autoRPM = false;

    private boolean autoAim = false;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, Alliance.BLUE);
    }

    @Override
    public void start() {
        robot.gate.closeGate();
        robot.hood.low();
    }

    @Override
    public void loop() {
        robot.drive(gamepad1);

        if (gamepad1.y) {
            robot.resetDrivePos();
        }

        if (gamepad2.left_bumper) robot.intakeIn();
        else if (gamepad2.left_trigger > 0.2) robot.intakeOut();
        else if (gamepad2.left_bumper && gamepad2.a) robot.intake.slowSpinIn();
        else robot.intakeOff();

        if (!autoRPM) {
            if (gamepad2.right_bumper) robot.shootHigh();
            else if (gamepad2.right_trigger > 0.2) robot.shootLow();
            else robot.stopShooter();
        } else robot.adjustSpeedAutomatically(robot.getDistanceFromTarget());


//        if (gamepad2.x && !lastX) autoAim = !autoAim;
//        lastX = gamepad2.x;

        if (gamepad2.x) robot.turret.setTurretTarget(0);

        if (autoAim) {
            robot.autoAim();
        }

//        if (Math.abs(gamepad2.left_stick_x) > 0.2) {
//            autoAim = false;
//            robot.manualTurret(gamepad2.left_stick_x * 0.2);
//        } else {
//            robot.autoTurret();
//        }

//        if (gamepad2.start) {
//            robot.turret.off();
//            robot.turret.resetTurret();
//            robot.turret.on();
//        }

        if (gamepad2.a) {
            robot.setUpRapidFire();
        }
        lastA = gamepad2.a;

        if (gamepad2.b && !lastB) robot.gate.toggle();
        lastB = gamepad2.b;

        if (gamepad2.y && !lastY) {
            autoRPM = !autoRPM;
            if (!autoRPM) robot.stopShooter();
        }
        lastY = gamepad2.y;


        if (gamepad2.dpad_up) robot.hood.moveUp();
        else if (gamepad2.dpad_down) robot.hood.moveDown();
        if (gamepad2.left_stick_y > 0.3) robot.hood.moveDown();
        if (gamepad2.left_stick_y < -0.3) robot.hood.moveUp();

//        boolean allowVision = autoAim || robot.shooterReady();

//        robot.turret.updateWithVisionAssist(allowVision);
        robot.periodic(false);

        updateTelemetry();
    }


    public void updateTelemetry() {
        telemetry.addData("Turret Ticks", robot.turret.getTurret());
        telemetry.addData("Turret Target", robot.turret.getTurretTarget());
        telemetry.addData("Turret Yaw (rad)", robot.turret.getYaw());
        telemetry.addData("Turret Power", robot.turret.power);
        telemetry.addData("Turret Mode", robot.turret.manual ? "Manual" : "Auto");
        telemetry.addData("Turret Degrees", Math.toDegrees(robot.turret.getYaw()));
        telemetry.addData("Ticks to Degrees", String.format("%.0f°", robot.turret.getTurret() * 0.374)); // 1° = 2.67 ticks
        telemetry.addData("Auto Aim", autoAim);

        // Shooter / Outtake
        telemetry.addData("Shooter Ready", robot.shooterReady());
        telemetry.addData("RPM Left", robot.outtake.getRPMLeft());
        telemetry.addData("RPM Right", robot.outtake.getRPMLeft());
        telemetry.addData("Left Ticks", robot.outtake.getTickLeft());
        telemetry.addData("Right Ticks", robot.outtake.getTickRight());
        telemetry.addData("Auto RPM", autoRPM);

        // Intake
        telemetry.addData("Intake Power", (gamepad2.left_bumper ? "In" : (gamepad2.left_trigger > 0.2 ? "Out" : "Off")));

        // Hood
        telemetry.addData("Right Servo Position", robot.hood.getRightPosition());
        telemetry.addData("Left Servo Position", robot.hood.getLeftPosition());

        // Drivetrain Pose
        telemetry.addData("Robot X", robot.getPose().getX());
        telemetry.addData("Robot Y", robot.getPose().getY());
        telemetry.addData("Robot Heading (rad)", robot.getPose().getHeading());
        telemetry.addData("Distance to goal", "%.1f in", robot.getDistanceFromTarget());
        telemetry.update();
    }
}
