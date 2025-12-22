package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "AutoV3", group = "Autonomous")
@Configurable
public class Auto extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower; // Pedro Pathing follower instance
    private int pathState;
    private DcMotorEx outtakeLeft, outtakeRight;
    private DcMotor intakeMotor;
    private Paths paths;

    private static final double TICKS_PER_REV = 28;
    private static final double TARGET_RPM_LOW = 3450;

    private long shootTimer = 0;
    private int shootStage = 0;
    private int ballsShot = 0;

    private boolean shootingActive = false;
    private boolean shootingComplete = false;
    private double lastTargetTicks = 0;

    @Override
    public void init() {
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        outtakeLeft = hardwareMap.get(DcMotorEx.class, "outtakeLeft");
        outtakeRight = hardwareMap.get(DcMotorEx.class, "outtakeRight");

        // PIDF
        double kP = 20, kI = 0, kD = 1.0;
        double kF = (32767.0 / (6000 * TICKS_PER_REV)) * 60.0;
        outtakeLeft.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        outtakeRight.setVelocityPIDFCoefficients(kP, kI, kD, kF);

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(21.46, 122.73, Math.toRadians(324)));

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        pathState = autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
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

    private void shootBalls() {
        long now = System.currentTimeMillis();

        switch (shootStage) {
            case 0:
                shootingActive = true;
                shootingComplete = false;
                shootTimer = now;
                shootStage = 1;
                break;

            case 1:
                if (now - shootTimer >= 200) startIntake();
                if (now - shootTimer >= 700) {
                    stopIntake();
                    shootTimer = now;
                    shootStage = 2;
                }
                break;

            case 2:
                if (now - shootTimer >= 150) {
                    startIntake();
                }
                if (now - shootTimer >= 300) {
                    stopIntake();
                    ballsShot++;
                    if (ballsShot >= 3) {
                        shootingActive = false;
                        shootingComplete = true;
                        shootStage = -1;
                    } else {
                        shootStage = 0;
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

    public static class Paths {

        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(24.400, 128.000), new Pose(50.000, 108.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-36), Math.toRadians(-90))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(50.000, 108.000),
                                    new Pose(50.000, 84.000),
                                    new Pose(19.000, 84.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(19.000, 84.000), new Pose(59.000, 84.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-90))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(59.000, 84.000),
                                    new Pose(59.000, 60.000),
                                    new Pose(19.000, 60.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(19.000, 60.000),
                                    new Pose(59.000, 60.000),
                                    new Pose(59.000, 84.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-90))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(59.000, 84.000),
                                    new Pose(59.000, 35.500),
                                    new Pose(19.000, 35.500)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(19.000, 35.500), new Pose(19.000, 72.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(19.000, 72.000), new Pose(15.000, 72.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();

            Path9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(15.000, 72.000), new Pose(62.000, 72.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (!follower.isBusy()) {
                    startOuttake();
                    follower.followPath(paths.Path1);
                    pathState = 1;
                }
                break;

            case 1:
                if (!follower.isBusy()) {
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

            case 2:
                if (!follower.isBusy()) {
                    startIntake();
                    follower.followPath(paths.Path2);
                    pathState = 3;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    stopIntake();
                    startOuttake();
                    follower.followPath(paths.Path3);
                    pathState = 4;
                }
                break;

            case 4:
                if (!follower.isBusy()) {
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
                        pathState = 5;
                    }
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    startIntake();
                    follower.followPath(paths.Path4);
                    pathState = 6;
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    stopIntake();
                    startOuttake();
                    follower.followPath(paths.Path5);
                    pathState = 7;
                }
                break;

            case 7:
                if (!follower.isBusy()) {
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
                        pathState = 8;
                    }
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    startIntake();
                    follower.followPath(paths.Path6);
                    pathState = 9;
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    stopIntake();
                    follower.followPath(paths.Path7);
                    pathState = 10;
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path8);
                    double stateTimer = System.currentTimeMillis();
                    //noinspection StatementWithEmptyBody
                    if (System.currentTimeMillis() - stateTimer <= 3000) {
                        // Do Nothing
                    }
                    pathState = 11;
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    startOuttake();
                    follower.followPath(paths.Path9);
                    pathState = 12;
                }
                break;

            case 12:
                if (!follower.isBusy()) {
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
                        pathState = -1;
                    }
                }
                break;

            default:
                stopIntake();
                stopOuttake();
                break;
        }

        return pathState;
    }
}