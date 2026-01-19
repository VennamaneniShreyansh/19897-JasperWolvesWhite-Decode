//package org.firstinspires.ftc.teamcode.references;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import org.firstinspires.ftc.teamcode.helper.Alliance;
//import org.firstinspires.ftc.teamcode.helper.Data;
//import org.firstinspires.ftc.teamcode.helper.ThreeBallShooterFar;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//import org.firstinspires.ftc.teamcode.subsystems.Robot;
//
//@Autonomous(name = "Blue-Auto Far 9 Artifact", group = "Autonomous")
//@Configurable
//@Disabled
//public class PedroAutonomousBlueFar extends OpMode {
//
//    private TelemetryManager panelsTelemetry;
//    public Follower follower;
//    private int pathState;
//    private Paths paths;
//    private ThreeBallShooterFar threeBallShooter;
//    private Robot robot;
//    private long shootStartTime = 0;
//    private long settleStartTime = 0;
//    private long stopCheckTime = 0;
//    private static final long SETTLE_MS = 2000;
//    private static final long SMALL_SETTLE_MS = 250;
//
//
//    @Override
//    public void init() {
//        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(new Pose(56, 8, Math.toRadians(180)));
//        paths = new Paths(follower);
//        robot = new Robot(hardwareMap, Alliance.BLUE);
//        threeBallShooter = new ThreeBallShooterFar(robot.intake, robot.outtake);
//
//        robot.outtake.periodic();
//        Data.setAutoPose(follower.getPose());
//
//        panelsTelemetry.debug("Status", "Initialized");
//        panelsTelemetry.update(telemetry);
//        pathState = 0;
//    }
//
//    @Override
//    public void start() {
//        robot.hood.set(.05, .95);
//        robot.outtake.periodic();
//    }
//
//    @Override
//    public void loop() {
//        follower.update();
//
//        robot.outtake.periodic();
//        robot.autoPeriodic();
//        threeBallShooter.update();
//
//        pathState = autonomousPathUpdate();
//
//        panelsTelemetry.debug("Path State", pathState);
//        panelsTelemetry.debug("Shooter Stage", threeBallShooter.stage);
//        panelsTelemetry.debug("Outtake RPM L", robot.outtake.getRPMLeft());
//        panelsTelemetry.debug("Outtake RPM R", robot.outtake.getRPMRight());
//        panelsTelemetry.debug("Outtake Target", robot.outtake.targetRPM);
//        panelsTelemetry.debug("At Target", robot.outtake.atTarget());
//        panelsTelemetry.debug("X", follower.getPose().getX());
//        panelsTelemetry.debug("Y", follower.getPose().getY());
//        panelsTelemetry.update(telemetry);
//    }
//
//    public int autonomousPathUpdate() {
//        switch (pathState) {
//            case 0: // Shoot 3
//                robot.turret.setTurretTarget(-187);
//                robot.outtake.shootHigh();
//                robot.gate.openGate();
//                if (!follower.isBusy()) {
//                    if (settleStartTime == 0) {
//                        settleStartTime = System.currentTimeMillis();
//                    }
//
//                    if (System.currentTimeMillis() - settleStartTime >= SETTLE_MS) {
//
//                        if (shootStartTime == 0) {
//                            threeBallShooter.start();
//                            shootStartTime = System.currentTimeMillis();
//                        }
//
//                        if (threeBallShooter.isDone()) {
//                            shootStartTime = 0;
//                            settleStartTime = 0;
//                            robot.gate.closeGate();
//                            robot.intakeIn();
//                            follower.setMaxPowerScaling(.75);
//                            follower.followPath(paths.Path1);
//                            pathState = 1;
//                        }
//                    }
//                }
//                break;
//
//            case 1: // From intake to Shoot 2
//                if (!follower.isBusy()) {
//                    if (stopCheckTime == 0) {
//                        stopCheckTime = System.currentTimeMillis();
//                    }
//
//                    if (System.currentTimeMillis() - stopCheckTime >= 500) {
//                        follower.setMaxPower(.5);
//                        follower.setMaxPowerScaling(.5);
//                        follower.followPath(paths.Path2);
//                        pathState = 2;
//                        stopCheckTime = 0;
//                        settleStartTime = 0;
//                    }
//                }
//                break;
//
//            case 2:
//                if (!follower.isBusy()) {
//                    if (settleStartTime == 0) {
//                        settleStartTime = System.currentTimeMillis();
//                        robot.intakeOff();
//                    }
//
//                    if (System.currentTimeMillis() - settleStartTime >= SMALL_SETTLE_MS) {
//                        robot.outtake.shootHigh();
//                        robot.gate.openGate();
//                        settleStartTime = 0;
//                        pathState = 3;
//                    }
//                }
//                break;
//
//            case 3: // Shoot 2nd 3
//                if (!follower.isBusy()) {
//                    if (settleStartTime == 0) {
//                        settleStartTime = System.currentTimeMillis();
//                    }
//
//                    if (System.currentTimeMillis() - settleStartTime >= SETTLE_MS) {
//
//                        if (shootStartTime == 0) {
//                            threeBallShooter.start();
//                            shootStartTime = System.currentTimeMillis();
//                        }
//
//                        if (threeBallShooter.isDone()) {
//                            shootStartTime = 0;
//                            settleStartTime = 0;
//                            robot.gate.closeGate();
//                            robot.intakeIn();
//                            follower.setMaxPower(1);
//                            follower.followPath(paths.Path3);
//                            stopCheckTime = System.currentTimeMillis();
//                            pathState = 4;
//                        }
//                    }
//                }
//                break;
//
//            case 4: // Intake to shoot
//                if (!follower.isBusy()) {
//                    if (stopCheckTime == 0) {
//                        stopCheckTime = System.currentTimeMillis();
//                    }
//
//                    if (System.currentTimeMillis() - stopCheckTime >= 500) {
//                        robot.intakeOff();
//                        follower.followPath(paths.Path4);
//                        if (System.currentTimeMillis() - stopCheckTime >= 450) {
//                            robot.outtake.shootHigh();
//                            robot.gate.openGate();
//                            pathState = 5;
//                            stopCheckTime = 0;
//                            settleStartTime = 0;
//                        }
//                    }
//                }
//                break;
//
//            case 5: // Shoot last 3
//                if (!follower.isBusy()) {
//                    if (settleStartTime == 0) {
//                        settleStartTime = System.currentTimeMillis();
//                    }
//
//                    if (System.currentTimeMillis() - settleStartTime >= SETTLE_MS) {
//
//                        if (shootStartTime == 0) {
//                            threeBallShooter.start();
//                            shootStartTime = System.currentTimeMillis();
//                        }
//
//                        if (threeBallShooter.isDone()) {
//                            shootStartTime = 0;
//                            settleStartTime = 0;
//                            robot.gate.closeGate();
//                            robot.stopShooter();
//                            robot.turret.setTurretTarget(0);
//                            Data.setAutoPose(follower.getPose());
//                            requestOpModeStop();
//                            stopCheckTime = 0;
//                        }
//                    }
//                }
//                break;
//        }
//
//        return pathState;
//    }
//
//    public static class Paths {
//        public PathChain Path1, Path2, Path3, Path4;
//
//        public Paths(Follower follower) {
//            Path1 = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(56.000, 8.000),
//                                    new Pose(8.000, 8.000)
//                            )
//                    ).setConstantHeadingInterpolation(Math.toRadians(180))
//                    .build();
//
//            Path2 = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(8.000, 8.000),
//                                    new Pose(53.000, 8.000)
//                            )
//                    ).setConstantHeadingInterpolation(Math.toRadians(180))
//                    .build();
//
//            Path3 = follower.pathBuilder().addPath(
//                            new BezierCurve(
//                                    new Pose(56.000, 8.000),
//                                    new Pose(56.673, 33.681),
//                                    new Pose(43.543, 36.186),
//                                    new Pose(15.900, 35.700)
//                            )
//                    ).setConstantHeadingInterpolation(Math.toRadians(180))
//                    .build();
//
//            Path4 = follower.pathBuilder().addPath(
//                            new BezierCurve(
//                                    new Pose(15.900, 35.700),
//                                    new Pose(43.543, 36.186),
//                                    new Pose(56.673, 33.681),
//                                    new Pose(56.000, 14.000)
//                            )
//                    ).setConstantHeadingInterpolation(Math.toRadians(180))
//                    .build();
//
////            Path5 = follower.pathBuilder().addPath(
////                            new BezierCurve(
////                                    new Pose(56.000, 8.000),
////                                    new Pose(56.000, 30.000)
////                            )
////                    ).setConstantHeadingInterpolation(Math.toRadians(180))
////                    .build(); Add the autonomousPathUpdate for this path
//        }
//    }
//}