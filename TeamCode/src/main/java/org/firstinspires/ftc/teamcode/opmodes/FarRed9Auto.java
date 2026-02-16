package org.firstinspires.ftc.teamcode.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.helper.Alliance;
import org.firstinspires.ftc.teamcode.helper.Data;
import org.firstinspires.ftc.teamcode.helper.FieldConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.helper.ThreeBallShooterFar;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "Far Red Auto 9", group = "Autonomous")
@Configurable
public class FarRed9Auto extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;
    private static Robot robot;
    private ThreeBallShooterFar threeBallShooter; // Add shooter
    private long shootStartTime = 0;
    private long settleStartTime = 0;
    private long stopCheckTime = 0;
    private static final long SETTLE_MS = 50;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56.5, 8.5, Math.toRadians(180)).mirror());

        paths = new Paths(follower);
        robot = new Robot(hardwareMap, Alliance.RED, false);

        // Set far goal target explicitly for autonomous
        robot.shootTarget = FieldConstants.farGoalPose(Alliance.RED);

        threeBallShooter = new ThreeBallShooterFar(robot.intake, robot.outtake);

        robot.outtake.periodic();
        Data.setAutoPose(follower.getPose());
        pathState = 0;

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }


    @Override
    public void start() {
        robot.hood.set(1, 0);
        robot.turret.setTurretTarget(-175);
        robot.turret.automatic();
        follower.setMaxPower(.7);
        Data.setAutoPose(follower.getPose());
    }


    @Override
    public void loop() {
        follower.update();
        robot.periodic();
        threeBallShooter.update();

        robot.autoAimWithFollower(follower.getPose());

        pathState = autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("Shooter Stage", threeBallShooter.stage);
        panelsTelemetry.debug("Outtake RPM L", robot.outtake.getRPMLeft());
        panelsTelemetry.debug("Outtake RPM R", robot.outtake.getRPMRight());
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("Turret Target", robot.turret.getTurretTarget());
        panelsTelemetry.debug("Turret Pos", robot.turret.getTurret());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {
        // Your existing paths (unchanged)
        public PathChain ToShoot, IntakeFirst, GoBack, IntakeSecond, ToShootTwo, LeaveLaunch,
                ToShootThree, IntakeThree;

        public Paths(Follower follower) {
            ToShoot = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(56.000, 8.500).mirror(),
                            new Pose(60.000, 15.000).mirror()
                    )
            ).setConstantHeadingInterpolation(Math.toRadians(0)).build();

            IntakeFirst = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(60.000, 15.000).mirror(),
                            new Pose(37.200, 6.700).mirror(),
                            new Pose(23.000, 7.500).mirror(),
                            new Pose(13.000, 8.000).mirror()
                    )
            ).setConstantHeadingInterpolation(Math.toRadians(0)).build();

            GoBack = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(13.000, 8.000).mirror(),
                            new Pose(20.000, 8.500).mirror()
                    )
            ).setConstantHeadingInterpolation(Math.toRadians(0)).build();

            IntakeSecond = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(20.000, 8.500).mirror(),
                            new Pose(13.000, 8.000).mirror()
                    )
            ).setConstantHeadingInterpolation(Math.toRadians(0)).build();

            ToShootTwo = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(13.000, 8.000).mirror(),
                                    new Pose(7.100, 29.000).mirror(),
                                    new Pose(12.200, 22.000).mirror(),
                                    new Pose(12.841, 4.644).mirror(),
                                    new Pose(60.000, 15.000).mirror()
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))
                    .addParametricCallback(0.5, () -> {
                        robot.intakeOff();
                    })
                    .addParametricCallback(.67, () -> {
                        robot.gate.openGate();
                    })
                    .build();

            IntakeThree = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(60.000, 15.000).mirror(),
                            new Pose(37.200, 6.700).mirror(),
                            new Pose(23.000, 7.500).mirror(),
                            new Pose(17.000, 8.000).mirror()
                    )
            ).setConstantHeadingInterpolation(Math.toRadians(0)).build();

            ToShootThree = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(17.000, 8.000).mirror(),
                                    new Pose(7.100, 29.000).mirror(),
                                    new Pose(12.200, 22.000).mirror(),
                                    new Pose(12.841, 4.644).mirror(),
                                    new Pose(60.000, 15.000).mirror()
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))
                    .addParametricCallback(0.5, () -> {
                        robot.intakeOff();
                    })
                    .addParametricCallback(.67, () -> {
                        robot.gate.openGate();
                    })
                    .build();

            LeaveLaunch = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(60.000, 15.000).mirror(),
                            new Pose(35.000, 8.000).mirror()
                    )
            ).setConstantHeadingInterpolation(Math.toRadians(0)).build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Drive to first shoot
                robot.gate.closeGate();
                follower.followPath(paths.ToShoot);
                robot.gate.openGate();
                pathState = 1;
                settleStartTime = 0;
                shootStartTime = 0;
                break;

            case 1: // Shoot first set (preload)
                if (!follower.isBusy()) {
                    if (settleStartTime == 0) {
                        settleStartTime = System.currentTimeMillis();
                    }

                    if (System.currentTimeMillis() - settleStartTime >= 50) {
                        if (shootStartTime == 0) {
                            threeBallShooter.start(true);
                            shootStartTime = System.currentTimeMillis();
                        }

                        if (threeBallShooter.isDone()) {
                            shootStartTime = 0;
                            settleStartTime = 0;

                            robot.gate.closeGate();
                            robot.intakeIn();
                            follower.followPath(paths.IntakeFirst);

                            pathState = 2;
                        }
                    }
                }
                break;

            case 2: // Intake first → drive back
                if (!follower.isBusy()) {
                    if (stopCheckTime == 0) {
                        stopCheckTime = System.currentTimeMillis();
                    }

                    if (System.currentTimeMillis() - stopCheckTime >= 300) {
//                        robot.intakeOff();
                        follower.followPath(paths.GoBack);

                        pathState = 3;
                        stopCheckTime = 0;
                        settleStartTime = 0;
                        shootStartTime = 0;
                    }
                }
                break;

            case 3: // Intake second
                if (!follower.isBusy()) {
                    robot.intakeIn();
                    follower.followPath(paths.IntakeSecond);

                    pathState = 4;
                    settleStartTime = 0;
                    shootStartTime = 0;
                }
                break;

            case 4: // Drive back to shoot
                if (!follower.isBusy()) {
                    follower.setMaxPower(.7);
                    robot.hood.set(1, 0);
                    follower.followPath(paths.ToShootTwo);

                    pathState = 5;
                    settleStartTime = 0;
                    shootStartTime = 0;
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    if (settleStartTime == 0) {
                        settleStartTime = System.currentTimeMillis();
                    }

                    if (System.currentTimeMillis() - settleStartTime >= SETTLE_MS) {
                        if (shootStartTime == 0) {
                            threeBallShooter.start(false);
                            shootStartTime = System.currentTimeMillis();
                        }

                        if (threeBallShooter.isDone()) {
                            shootStartTime = 0;
                            settleStartTime = 0;

                            robot.gate.closeGate();
                            robot.intakeIn();
                            follower.followPath(paths.IntakeThree);

                            pathState = 8;
                        }
                    }
                }
                break;

            case 6: // Intake first → drive back
                if (!follower.isBusy()) {
                    if (stopCheckTime == 0) {
                        stopCheckTime = System.currentTimeMillis();
                    }

                    if (System.currentTimeMillis() - stopCheckTime >= 300) {
//                        robot.intakeOff();
                        follower.followPath(paths.GoBack);

                        pathState = 7;
                        stopCheckTime = 0;
                        settleStartTime = 0;
                        shootStartTime = 0;
                    }
                }
                break;

            case 7: // Intake second
                if (!follower.isBusy()) {
                    robot.intakeIn();
                    follower.followPath(paths.IntakeSecond);

                    pathState = 8;
                    settleStartTime = 0;
                    shootStartTime = 0;
                }
                break;

            case 8: // Drive back to shoot
                if (!follower.isBusy()) {
                    follower.setMaxPower(.7);
//                    robot.turret.setTurretTarget(-172);
                    follower.followPath(paths.ToShootThree);

                    pathState = 9;
                    settleStartTime = 0;
                    shootStartTime = 0;
                }
                break;

            case 9: // Shoot second set
                if (!follower.isBusy()) {
                    if (settleStartTime == 0) {
                        settleStartTime = System.currentTimeMillis();
                    }

                    if (System.currentTimeMillis() - settleStartTime >= SETTLE_MS) {
                        if (shootStartTime == 0) {
                            threeBallShooter.start(false);
                            shootStartTime = System.currentTimeMillis();
                        }

                        if (threeBallShooter.isDone()) {
                            shootStartTime = 0;
                            settleStartTime = 0;

//                            robot.gate.closeGate();
                            robot.stopShooter();
                            follower.followPath(paths.LeaveLaunch);

                            pathState = 10;
                        }
                    }
                }
                break;

            case 10: // Leave and finish
                if (!follower.isBusy()) {
//                    robot.turret.setTurretTarget(0);
//                    robot.turret.setTurretTarget(0);
                    Data.setAutoPose(follower.getPose());
                }
                break;
        }
        return pathState;
    }
}
