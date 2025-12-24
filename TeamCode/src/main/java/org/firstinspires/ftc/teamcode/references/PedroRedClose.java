package org.firstinspires.ftc.teamcode.references;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Pedro Auto Red MFITBILTPDSPOTBATBW", group = "Autonomous")
@Configurable // Panels
@Disabled
public class PedroRedClose extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private DcMotorEx outtakeLeft, outtakeRight;
    private DcMotor intakeMotor;
    private Servo servo;
    private static final double TICKS_PER_REV = 28;
    private static final double TARGET_RPM_LOW = 2650;
    private long shootTimer = 0;
    private int shootStage = 0;
    private int ballsShot = 0;
    private static final long BALL_TIMEOUT_MS = 2000;
    private double stateTimer = 0;

    // shooting state tracking for autonomous transitions
    private boolean shootingActive = false;
    private boolean shootingComplete = false;

    private double lastTargetTicks = 0;


    @Override
    public void init() {
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        outtakeLeft = hardwareMap.get(DcMotorEx.class, "outtakeLeft");
        outtakeRight = hardwareMap.get(DcMotorEx.class, "outtakeRight");
        servo = hardwareMap.get(Servo.class, "servo");

        // PIDF
        double kP = 20, kI = 0, kD = 1.0;
        double kF = (32767.0 / (6000 * TICKS_PER_REV)) * 60.0;
        outtakeLeft.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        outtakeRight.setVelocityPIDFCoefficients(kP, kI, kD, kF);

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(21.46, 122.73, Math.toRadians(324)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public void stopOuttake() {
        outtakeLeft.setVelocity(0);
        outtakeRight.setVelocity(0);
        lastTargetTicks = 0;
    }

    private void startOuttake() {
        double targetTicks = TARGET_RPM_LOW * TICKS_PER_REV / 60.0;

        if (targetTicks != lastTargetTicks) {
            // original forward directions (left negative, right positive)
            outtakeLeft.setVelocity(-targetTicks);
            outtakeRight.setVelocity(targetTicks);

            lastTargetTicks = targetTicks;
        }
    }

    public static Pose mirror(Pose p) {
        double newX = 144 - p.getX();               // Mirror X across midline
        double newY = p.getY();                     // Y stays the same
        double newHeading = Math.toRadians(180) - p.getHeading();
        return new Pose(newX, newY, newHeading);
    }

    private void startIntake() {
        double INTAKE_POWER = 1;
        intakeMotor.setPower(INTAKE_POWER);
    }

    private void stopIntake() {
        intakeMotor.setPower(0);
    }

    private void openServo() {
//        double OPEN_SERVO = 0.15;
//        servo.setPosition(OPEN_SERVO);
    }
    private void closeServo() {
//        double CLOSE_SERVO = 0.0;
//        servo.setPosition(CLOSE_SERVO);
    }


//    private void shootBalls() {
//        long now = System.currentTimeMillis();
//
//        switch (shootStage) {
//            case 0:
//                // Start new cycle
//                shootingActive = true;
//                shootingComplete = false;
//                closeServo();                   // Step 1: close servo right away
//                shootTimer = now;
//                shootStage = 1;
//                break;
//
//            case 1:
//
//                if (now - shootTimer >= 200) { // after 200 ms
//                    startIntake();
//                }
//                // Step 2: intake towards the servo for 500 ms
//                if (now - shootTimer >= 700) {
//                    stopIntake();
//                    shootTimer = now;
//                    shootStage = 2;
//                }
//                break;
//
//            case 2:
//                // Step 3: wait 100 ms, then open servo to shoot
//                if (now - shootTimer >= 150) {
//                    openServo();                // Step 4: fire ball
//                    shootTimer = now;
//                    shootStage = 3;
//                }
//                break;
//
//            case 3:
//                // Step 5: intake briefly to send the ball through after opening
//                if (now - shootTimer >= 305) {
//                    startIntake();
//                }
//                if (now - shootTimer >= 405) {  // intake for ~150 ms
//                    stopIntake();               // Step 6: stop intake
//                    closeServo();
//                    ballsShot++;
//                    if (now - shootTimer >= 1750){
//                        startIntake();
//                        if (now - shootTimer >= 2000) {       // after 3 cycles, finish
//                            stopIntake();
//                            shootingActive = false;
//                            shootingComplete = true;
//                            shootStage = 999;
//                        }
//                    }
//                }
//                break;
//
//            default:
//                // Finished shooting
//                stopIntake();
//                closeServo();
//                break;
//        }
//    }

    private void shootBalls() {
        long now = System.currentTimeMillis();

        switch (shootStage) {
            case 0:
                // Start new cycle
                shootingActive = true;
                shootingComplete = false;
                shootTimer = now;
                shootStage = 1;
                break;

            case 1:

                if (now - shootTimer >= 200) {
                    startIntake();
                }
                // Step 2: intake towards the servo for 500 ms
                if (now - shootTimer >= 500) {
                    stopIntake();
                    shootTimer = now;
                    shootStage = 2;
                }
                break;

            case 2:
                // Step 3: wait 100 ms, then open servo to shoot
                if (now - shootTimer >= 1000) {
                    startIntake();
                    shootTimer = now;
                    shootStage = 3;
                }
                break;

            case 3:
                if (now - shootTimer >= 1000) {
                    stopIntake();
                    shootingActive = false;
                    shootingComplete = true;
                    shootStage = 9999;
                }
                break;
            default:
                // Finished shooting
                stopIntake();
                stopOuttake();
                break;
        }
    }
    public static class Paths {

        public PathChain ToShoot;
        public PathChain ToFirstSet;
        public PathChain IntakeFirstSet;
        public PathChain ToShoot2;
        public PathChain ToSecondSet;
        public PathChain IntakeSecondSet;
        public PathChain ToShoot3;
        public PathChain ToThirdSet;
        public PathChain IntakeSetThird;

        public Paths(Follower follower) {
            ToShoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(122.537, 122.732), new Pose(91.000, 90.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(216), Math.toRadians(222))
                    .build();

            ToFirstSet = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(91.000, 90.000),
                                    new Pose(88.585, 83.707),
                                    new Pose(99.000, 84.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(222), Math.toRadians(0))
                    .build();

            IntakeFirstSet = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(99.000, 84.000), new Pose(124.500, 84.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            ToShoot2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(124.500, 84.000), new Pose(91.000, 90.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(222))
                    .build();

            ToSecondSet = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(91.000, 90.000),
                                    new Pose(85.000, 71.500),
                                    new Pose(94.000, 60.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(222), Math.toRadians(0))
                    .build();

            IntakeSecondSet = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(94.000, 60.000), new Pose(120.000, 60.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            ToShoot3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(120.000, 60.000), new Pose(91.000, 90.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(222))
                    .build();

            ToThirdSet = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(91.000, 90.000), new Pose(94.000, 35.500))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(216), Math.toRadians(0))
                    .build();

            IntakeSetThird = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(94.000, 35.500), new Pose(129.000, 36.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }



    public int autonomousPathUpdate() {

        switch (pathState) {

            case 0:
                // Start outtake
                stateTimer = System.currentTimeMillis();
                startOuttake();
                follower.setMaxPowerScaling(.85);
                follower.followPath(paths.ToShoot);
                pathState = 1;
                break;

            case 1:
                if (!follower.isBusy() && System.currentTimeMillis() - stateTimer >= 4000) {
                    if (!shootingActive && !shootingComplete) {
                        follower.setMaxPowerScaling(1);
                        shootStage = 0;
                        ballsShot = 0;
                        shootTimer = System.currentTimeMillis();
                        shootingActive = true;
                        closeServo();
                    }
                    else if (shootingActive && !shootingComplete) {
                        shootBalls();
                    } else if (shootingComplete) {
                        shootingComplete = false;
                        pathState = 2;
                    }
                }
                break;

            case 2:
                // Drive to first set,  close servo, stop outtake and intake
                stopOuttake();
                closeServo();
                follower.setMaxPowerScaling(.7);
                follower.followPath(paths.ToFirstSet);
                pathState = 3;
                break;

            case 3:
                if (!follower.isBusy()) {
                    // Intake on to intake the  first set,  keep the servo closed
                    follower.setMaxPowerScaling(.8);
                    startIntake();
                    closeServo();
                    follower.followPath(paths.IntakeFirstSet);
                    pathState = 4;
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.setMaxPowerScaling(1);
                    // Turn off intake, Turn on outtake while going to shoot, keep the servo closed
                    closeServo();
                    startOuttake();
                    stopIntake();
                    follower.followPath(paths.ToShoot2);
                    pathState = 5;
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    // Same shooting behavior as case 1
                    if (!shootingActive && !shootingComplete) {
                        follower.setMaxPowerScaling(1);
                        shootStage = 0;
                        ballsShot = 0;
                        shootTimer = System.currentTimeMillis();
                        shootingActive = true;
                        closeServo();
                    }
                    else if (shootingActive && !shootingComplete) {
                        shootBalls();
                    } else if (shootingComplete) {
                        shootingComplete = false;
                        follower.followPath(paths.ToSecondSet);
                        // Close servo, Outtake off
                        stopOuttake();
                        closeServo();
                        pathState = 6;
                    }
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    // Turn intake on, keep servo closed
                    follower.setMaxPowerScaling(.8);
                    startIntake();
                    closeServo();
                    follower.followPath(paths.IntakeSecondSet);
                    pathState = 7;
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.setMaxPowerScaling(1);
                    // Stop intake, keep outtake on, keep servo closed
                    closeServo();
                    startOuttake();
                    stopIntake();
                    follower.followPath(paths.ToShoot3);
                    pathState = 8;
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    // Same shooting behavior as case 1/5
                    if (!shootingActive && !shootingComplete) {
                        follower.setMaxPowerScaling(1);
                        shootStage = 0;
                        ballsShot = 0;
                        shootTimer = System.currentTimeMillis();
                        shootingActive = true;
                        closeServo();
                    }
                    else if (shootingActive && !shootingComplete) {
                        shootBalls();
                    } else if (shootingComplete) {
                        shootingComplete = false;
                        follower.followPath(paths.ToThirdSet);
                        // Close Servo, turn off intake and outtake
                        closeServo();
                        stopOuttake();
                        pathState = 9;
                    }
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    // Keep servo closed, turn on intake
                    closeServo();
                    startIntake();
                    stopOuttake();
                    follower.followPath(paths.IntakeSetThird);
                    pathState = 10;
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    // Intake off
                    stopIntake();
                    pathState = 67;
                }
                break;

            default:
                // Stop moving and finish auto
                break;
        }

        return pathState;
    }
}