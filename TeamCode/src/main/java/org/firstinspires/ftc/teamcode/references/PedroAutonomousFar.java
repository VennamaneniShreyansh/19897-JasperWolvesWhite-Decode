package org.firstinspires.ftc.teamcode.references;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Pedro Auto Far MFITBILTPDSPOTBATBW", group = "Autonomous")
@Configurable
@Disabled
public class PedroAutonomousFar extends OpMode {

    private TelemetryManager panelsTelemetry;
    private int pathState;
    private DcMotorEx outtakeLeft, outtakeRight;
    private DcMotor intakeMotor;
    private Servo servo;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private static final double TICKS_PER_REV = 28;
    private static final double TARGET_RPM_LOW = 3450;

    private long shootTimer = 0;
    private int shootStage = 0;
    private int ballsShot = 0;

    private double stateTimer = 0;

    private boolean shootingActive = false;
    private boolean shootingComplete = false;
    private double lastTargetTicks = 0;

    @Override
    public void init() {
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        outtakeLeft = hardwareMap.get(DcMotorEx.class, "outtakeLeft");
        outtakeRight = hardwareMap.get(DcMotorEx.class, "outtakeRight");
        servo = hardwareMap.get(Servo.class, "servo");

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // PIDF
        double kP = 20, kI = 0, kD = 1.0;
        double kF = (32767.0 / (6000 * TICKS_PER_REV)) * 60.0;
        outtakeLeft.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        outtakeRight.setVelocityPIDFCoefficients(kP, kI, kD, kF);

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(new Pose(21.46, 122.73, Math.toRadians(324)));


        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
//        follower.update();
        pathState = autonomousPathUpdate();

//        panelsTelemetry.debug("Path State", pathState);
//        panelsTelemetry.debug("X", follower.getPose().getX());
//        panelsTelemetry.debug("Y", follower.getPose().getY());
//        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
//        panelsTelemetry.update(telemetry);
    }

    private void startOuttake() {
        double targetTicks = TARGET_RPM_LOW * TICKS_PER_REV / 60.0;
        if (targetTicks != lastTargetTicks) {
            outtakeLeft.setVelocity(-targetTicks);
            outtakeRight.setVelocity(targetTicks);
            lastTargetTicks = targetTicks;
        }
    }

    private void stopOuttake() {
        outtakeLeft.setVelocity(0);
        outtakeRight.setVelocity(0);
        lastTargetTicks = 0;
    }

    private void startIntake() {
        intakeMotor.setPower(1);
    }

    private void stopIntake() {
        intakeMotor.setPower(0);
    }

    private void openServo() {
//        servo.setPosition(0.15);
    }

    private void closeServo() {
//        servo.setPosition(0.0);
    }

    private void shootBalls() {
        long now = System.currentTimeMillis();

        switch (shootStage) {
            case 0: // Start
                shootingActive = true;
                shootingComplete = false;
                closeServo();
                shootTimer = now;
                shootStage = 1;
                break;

            case 1: // Intake toward servo
                if (now - shootTimer >= 200) startIntake();
                if (now - shootTimer >= 700) {
                    stopIntake();
                    shootTimer = now;
                    shootStage = 2;
                }
                break;

            case 2: // Fire
                if (now - shootTimer >= 150) {
                    openServo();
                    shootTimer = now;
                    shootStage = 3;
                }
                break;

            case 3: // Finish shot
                if (now - shootTimer >= 150) {
                    startIntake();
                }
                if (now - shootTimer >= 300) {
                    stopIntake();
                    closeServo();
                    ballsShot++;
                    if (ballsShot >= 3) { // 3 balls total
                        shootingActive = false;
                        shootingComplete = true;
                        shootStage = 999;
                    } else {
                        shootStage = 0; // next ball
                    }
                    shootTimer = now;
                }
                break;

            default:
                stopIntake();
                stopOuttake();
                break;
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Start outtake and move
                startOuttake();
//                follower.setMaxPowerScaling(.85);
//                follower.followPath(paths.ToShoot);
                stateTimer = System.currentTimeMillis();
                pathState = 1;
                break;

            case 1: // Wait, then shoot
                if (System.currentTimeMillis() - stateTimer >= 4000) {
                    if (!shootingActive && !shootingComplete) {
                        shootingActive = true;
                        shootStage = 0;
                        ballsShot = 0;
                        shootTimer = System.currentTimeMillis();
                    }
                    if (shootingActive) {
                        shootBalls();
                    }
                    if (shootingComplete) {
                        pathState = 2;
                    }
                }
                break;

            case 2: // Done
                stopIntake();
                stopOuttake();
                pathState = 3;
                break;

            case 3:
                moveForward();
                stateTimer = System.currentTimeMillis();
                pathState = 4;
            case 4:
                if (System.currentTimeMillis() - stateTimer >= 2000) {
                   stopMotors();
                }

            default:
                stopIntake();
                stopOuttake();
                break;
        }

        return pathState;
    }

    private void moveForward() {
        frontLeft.setPower(-.6);

        backLeft.setPower(-.6);

        frontRight.setPower(-.6);

        backRight.setPower(-.6);
    }



    private void stopMotors() {
        frontLeft.setPower(0);

        backLeft.setPower(0);

        frontRight.setPower(0);

        backRight.setPower(0);
    }

}
