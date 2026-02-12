package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.helper.Alliance;

@TeleOp(name = "Main TeleOp", group = "TeleOp")
public class Teleop extends OpMode {

    private Robot robot;
    double loopStart = System.nanoTime();
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;
    private boolean autoRPM = false;

    private boolean autoAim = true;

    @Override
    public void init() {

        telemetry.addLine("Select Alliance:");
        telemetry.addLine("X = BLUE");
        telemetry.addLine("Y = RED");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        boolean alrDone = false;
        if (alrDone) {
            telemetry.addData("Status", "Robot initialized");
            telemetry.update();
            return;  // Exit early after init
        }

        Alliance alliance = null;
        if (gamepad1.x) {
            alliance = Alliance.BLUE;
        } else if (gamepad1.y) {
            alliance = Alliance.RED;
        }

        if (alliance != null) {
            alrDone = true;
            robot = new Robot(hardwareMap, alliance);
            telemetry.addData("Selected", alliance.name());
        } else {
            telemetry.addData("Press", "X=Blue, Y=Red");
        }

        telemetry.addData("alrDone", alrDone);
        telemetry.update();
    }


    @Override
    public void start() {
        robot.resetDrivePosAtGoal();
        robot.gate.closeGate();
        robot.hood.high();
    }

    @Override
    public void loop() {
        // Drivetrain
        robot.drive(gamepad1);
        if (gamepad1.a) robot.resetDrivePosAtGoal();
        // Manuel Turret
        if (!autoAim && gamepad1.dpadLeftWasPressed()) {
            robot.turret.incrementTurretRight();
        } else if (!autoAim && gamepad1.dpadRightWasPressed()) {
            robot.turret.incrementTurretLeft();
        } else if (!autoAim && (gamepad1.yWasPressed())) {
            robot.turret.resetTurret();
        }
        // Intake
        if (gamepad2.left_bumper && (gamepad2.right_trigger > .1 || gamepad2.right_bumper)) robot.slowIntakeIn();
        else if (gamepad2.left_bumper) robot.intakeIn();
        else if (gamepad2.left_trigger > 0.2) robot.intakeOut();
        else robot.intakeOff();
        // Shooting
        if (!autoRPM) {
            if (gamepad2.right_bumper) robot.shootHigh();
            else if (gamepad2.right_trigger > 0.2) robot.shootLow();
            else robot.stopShooter();
        } else robot.adjustSpeedAutomatically(robot.getDistanceFromTarget());
        // Auto Aim
        if (gamepad2.x && !lastX) autoAim = !autoAim;
        lastX = gamepad2.x;
        if (gamepad2.x) robot.stopTurretAim();

        if (autoAim) {
            robot.autoAim();
        }


        if (gamepad2.b && !lastB) robot.gate.toggle();
        lastB = gamepad2.b;

        if (gamepad2.y && !lastY) {
            autoRPM = !autoRPM;
            if (!autoRPM) robot.stopShooter();
        }
        lastY = gamepad2.y;


        if (gamepad2.dpadUpWasPressed()) robot.hood.moveUp();
        else if (gamepad2.dpadDownWasPressed()) robot.hood.moveDown();

        robot.periodic();

        updateTelemetry();
    }


    public void updateTelemetry() {
        telemetry.addData("Turret Ticks", robot.turret.getTurret());
        telemetry.addData("Turret Target", robot.turret.getTurretTarget());
        telemetry.addData("Turret Degrees", Math.toDegrees(robot.turret.getYaw()));
        telemetry.addData("Auto Aim", autoAim);

        telemetry.addData("RPM Left", robot.outtake.getRPMLeft());
        telemetry.addData("RPM Right", robot.outtake.getRPMLeft());
        telemetry.addData("Left Ticks", robot.outtake.getTickLeft());
        telemetry.addData("Right Ticks", robot.outtake.getTickRight());
        telemetry.addData("Auto RPM", autoRPM);

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
