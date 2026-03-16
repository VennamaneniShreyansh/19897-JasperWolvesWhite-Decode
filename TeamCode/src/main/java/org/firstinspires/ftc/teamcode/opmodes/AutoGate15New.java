package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.helper.Alliance;
import org.firstinspires.ftc.teamcode.helper.Data;
import org.firstinspires.ftc.teamcode.helper.RapidFire;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous(name = "Blue Auto RapidFire 15 Gate", group = "Autonomous")
@Configurable
@Config
public class AutoGate15New extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;
    private Robot robot;
    private RapidFire threeBallShooter;
    private long shootStartTime = 0;
    private long settleStartTime = 0;
    private long stopCheckTime = 0;
    private static final long SETTLE_MS = 50;

    public void drawRobot(Canvas canvas, Pose pose) {
        Pose newPose = PedroToFTC(pose.getX(), pose.getY(), pose.getHeading());
        canvas.strokeCircle(newPose.getX(), newPose.getY(), 9);
        Vector v = newPose.getHeadingAsUnitVector().times(9);
        double x1 = newPose.getX() + v.getXComponent() / 2, y1 = newPose.getY() + v.getYComponent() / 2;
        double x2 = newPose.getX() + v.getXComponent(), y2 = newPose.getY() + v.getYComponent();
        canvas.strokeLine(x1, y1, x2, y2);
    }

    public double angleWrap(double heading) {
        while (heading >= Math.PI) {
            heading -= Math.PI;
        }
        while (heading < -Math.PI) {
            heading += Math.PI;
        }
        return heading;
    }

    public Pose PedroToFTC(double pedroX, double pedroY, double pedroHeading) {
        double ftcY = -72 + pedroX;
        double ftcX = 72 - pedroY;
        double ftcHeading = pedroHeading + Math.PI / 2;
        ftcHeading %= 2 * Math.PI;
        ftcHeading = angleWrap(ftcHeading);
        return new Pose(ftcX, ftcY, ftcHeading);
    }

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(33.75, 135.5, Math.toRadians(180)));
        robot = new Robot(hardwareMap, Alliance.BLUE, false);
        paths = new Paths(follower);
        threeBallShooter = new RapidFire(robot.intake, robot.outtake);

        Data.setAutoPose(follower.getPose());

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
        pathState = 0;
    }

    @Override
    public void start() {
        robot.hood.set(.9, .1);
        robot.outtake.periodic();
    }

    @Override
    public void loop() {
        follower.update();
        robot.periodic();
        threeBallShooter.update();

        Data.setAutoPose(follower.getPose());

        pathState = autonomousPathUpdate();
        robot.shootHigh();

        try {
            TelemetryPacket packet = new TelemetryPacket();
            drawRobot(packet.fieldOverlay(), follower.getPose());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        } catch (RuntimeException e) {
            e.printStackTrace();
        }
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("Shooter Stage", threeBallShooter.stage);
        panelsTelemetry.debug("Outtake RPM L", robot.outtake.getRPMLeft());
        panelsTelemetry.debug("Outtake RPM R", robot.outtake.getRPMRight());
        panelsTelemetry.debug("Shooter enabled", robot.outtake.isEnabled());
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.update(telemetry);
    }

    public int autonomousPathUpdate() {
        switch (pathState) {

            case 0: // Go to First Shoot
                robot.turret.setTurretTarget(-45);
                if (!follower.isBusy()) {
                    robot.gate.openGate();
                    follower.followPath(paths.ToShoot);
                    robot.gate.openGate();
                    pathState = 1;
                    settleStartTime = 0;
                }
                break;

            case 1: // Shoot 1 → IntakeSecondSet
                if (!follower.isBusy()) {
                    if (settleStartTime == 0) {
                        settleStartTime = System.currentTimeMillis();
                    }
                    if (System.currentTimeMillis() - settleStartTime >= 50) {
                        if (shootStartTime == 0) {
                            threeBallShooter.start();
                            shootStartTime = System.currentTimeMillis();
                        }
                        if (threeBallShooter.isDone()) {
                            shootStartTime = 0;
                            settleStartTime = 0;
                            robot.gate.closeGate();
                            robot.intakeIn();
                            follower.setMaxPower(1);
                            follower.followPath(paths.IntakeSecondSet);
                            pathState = 2;
                        }
                    }
                }
                break;

            case 2: // IntakeSecondSet → Shoot2
                if (!follower.isBusy()) {
                    if (stopCheckTime == 0) {
                        stopCheckTime = System.currentTimeMillis();
                    }
                    if (System.currentTimeMillis() - stopCheckTime >= 300) {
                        robot.intakeOff();
                        robot.gate.openGate();
                        follower.followPath(paths.ToShoot2);
                        pathState = 3;
                        stopCheckTime = 0;
                        settleStartTime = 0;
                    }
                }
                break;

            case 3: // Shoot 2 → OpenGate
                if (!follower.isBusy()) {
                    if (settleStartTime == 0) {
                        settleStartTime = System.currentTimeMillis();
                    }
                    if (System.currentTimeMillis() - settleStartTime >= SETTLE_MS) {
                        if (shootStartTime == 0) {
                            threeBallShooter.start();
                            shootStartTime = System.currentTimeMillis();
                        }
                        if (threeBallShooter.isDone()) {
                            shootStartTime = 0;
                            settleStartTime = 0;
                            robot.gate.closeGate();
                            robot.intakeIn();
                            follower.setMaxPower(.9);
                            follower.followPath(paths.OpenGate);
                            pathState = 4;
                        }
                    }
                }
                break;

            case 4: // OpenGate → IntakeFromGate
                if (!follower.isBusy()) {
                    if (stopCheckTime == 0) {
                        stopCheckTime = System.currentTimeMillis();
                    }
                    if (System.currentTimeMillis() - stopCheckTime >= 300) {
                        robot.intakeIn();
                        follower.setMaxPower(.7);
                        follower.followPath(paths.IntakeFromGate);
                        pathState = 5;
                        stopCheckTime = 0;
                        settleStartTime = 0;
                    }
                }
                break;

            case 5: // IntakeFromGate → Shoot3
                if (!follower.isBusy()) {
                    if (stopCheckTime == 0) {
                        stopCheckTime = System.currentTimeMillis();
                    }
                    if (System.currentTimeMillis() - stopCheckTime >= 1000) {
                        robot.intakeOff();
                        robot.gate.openGate();
                        follower.setMaxPower(.9);
                        follower.followPath(paths.ToShoot3);
                        pathState = 6;
                        stopCheckTime = 0;
                        settleStartTime = 0;
                    }
                }
                break;

            case 6: // Shoot 3 → OpenGate2
                if (!follower.isBusy()) {
                    if (settleStartTime == 0) {
                        settleStartTime = System.currentTimeMillis();
                    }
                    if (System.currentTimeMillis() - settleStartTime >= SETTLE_MS) {
                        if (shootStartTime == 0) {
                            threeBallShooter.start();
                            shootStartTime = System.currentTimeMillis();
                        }
                        if (threeBallShooter.isDone()) {
                            shootStartTime = 0;
                            settleStartTime = 0;
                            robot.gate.closeGate();
                            robot.intakeIn();
                            follower.setMaxPower(.9);
                            follower.followPath(paths.OpenGate2);
                            pathState = 7;
                        }
                    }
                }
                break;

            case 7: // OpenGate2 → IntakeFromGate2
                if (!follower.isBusy()) {
                    if (stopCheckTime == 0) {
                        stopCheckTime = System.currentTimeMillis();
                    }
                    if (System.currentTimeMillis() - stopCheckTime >= 300) {
                        robot.intakeIn();
                        follower.setMaxPower(.9);
                        follower.followPath(paths.IntakeFromGate2);
                        pathState = 8;
                        stopCheckTime = 0;
                        settleStartTime = 0;
                    }
                }
                break;

            case 8: // IntakeFromGate2 → Shoot4
                if (!follower.isBusy()) {
                    if (stopCheckTime == 0) {
                        stopCheckTime = System.currentTimeMillis();
                    }
                    if (System.currentTimeMillis() - stopCheckTime >= 1000) {
                        robot.intakeOff();
                        robot.gate.openGate();
                        follower.followPath(paths.ToShoot4);
                        pathState = 9;
                        stopCheckTime = 0;
                        settleStartTime = 0;
                    }
                }
                break;

            case 9: // Shoot 4 → IntakeFirstSet
                if (!follower.isBusy()) {
                    if (settleStartTime == 0) {
                        settleStartTime = System.currentTimeMillis();
                    }
                    if (System.currentTimeMillis() - settleStartTime >= SETTLE_MS) {
                        if (shootStartTime == 0) {
                            threeBallShooter.start();
                            shootStartTime = System.currentTimeMillis();
                        }
                        if (threeBallShooter.isDone()) {
                            robot.gate.closeGate();
                            shootStartTime = 0;
                            settleStartTime = 0;
                            robot.intakeIn();
                            follower.setMaxPower(.9);
                            follower.followPath(paths.IntakeFirstSet);
                            pathState = 10;
                        }
                    }
                }
                break;

            case 10: // IntakeFirstSet → ToShoot5
                if (!follower.isBusy()) {
                    if (stopCheckTime == 0) {
                        stopCheckTime = System.currentTimeMillis();
                    }
                    if (System.currentTimeMillis() - stopCheckTime >= 300) {
                        robot.intakeOff();
                        robot.turret.setTurretTarget(-26);
                        robot.gate.openGate();
                        follower.followPath(paths.ToShoot5);
                        pathState = 11;
                        stopCheckTime = 0;
                        settleStartTime = 0;
                    }
                }
                break;

            case 11: // Final Shoot 5 → Park
                if (!follower.isBusy()) {
                    if (settleStartTime == 0) {
                        settleStartTime = System.currentTimeMillis();
                    }
                    if (System.currentTimeMillis() - settleStartTime >= SETTLE_MS) {
                        if (shootStartTime == 0) {
                            threeBallShooter.start();
                            shootStartTime = System.currentTimeMillis();
                        }
                        if (threeBallShooter.isDone()) {
                            shootStartTime = 0;
                            settleStartTime = 0;
                            robot.gate.closeGate();
                            robot.stopShooter();
                            pathState = 12;
                        }
                    }
                }
                break;

            case 12: // Park
                if (!follower.isBusy()) {
                    if (stopCheckTime == 0) {
                        stopCheckTime = System.currentTimeMillis();
                    }
                    if (System.currentTimeMillis() - stopCheckTime >= 50) {
                        robot.intakeOff();
                        robot.turret.setTurretTarget(0);
                        Data.setAutoPose(follower.getPose());
                        requestOpModeStop();
                        stopCheckTime = 0;
                    }
                }
                break;
        }
        return pathState;
    }


    public static class Paths {
        public PathChain ToShoot;
        public PathChain IntakeSecondSet;
        public PathChain ToShoot2;
        public PathChain OpenGate;
        public PathChain IntakeFromGate;
        public PathChain ToShoot3;
        public PathChain OpenGate2;
        public PathChain IntakeFromGate2;
        public PathChain ToShoot4;
        public PathChain IntakeFirstSet;
        public PathChain ToShoot5;

        public Paths(Follower follower) {
            ToShoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(33.756, 135.220),

                                    new Pose(56.000, 88.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            IntakeSecondSet = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(56.000, 88.000),
                                    new Pose(48.873, 63.967),
                                    new Pose(43.498, 58.458),
                                    new Pose(20.000, 59.848)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            ToShoot2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(20.000, 59.848),

                                    new Pose(56.000, 88.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            OpenGate = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(56.000, 88.000),
                                    new Pose(38.200, 57.900),
                                    new Pose(17.500, 65.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(157))
                    .addParametricCallback(0.5, () -> follower.setMaxPower(0.5))
                    .build();

            IntakeFromGate = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(17.500, 65.000),

                                    new Pose(13.000, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(157), Math.toRadians(120))

                    .build();

            ToShoot3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(13.000, 60.000),
                                    new Pose(39.000, 72.600),
                                    new Pose(56.000, 88.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(180))

                    .build();

            OpenGate2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(56.000, 88.000),
                                    new Pose(38.200, 57.900),
                                    new Pose(17.500, 65.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(157))
                    .addParametricCallback(0.5, () -> follower.setMaxPower(0.5))
                    .build();

            IntakeFromGate2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(17.500, 65.000),

                                    new Pose(13.000, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(157), Math.toRadians(130))

                    .build();

            ToShoot4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(13.000, 60.000),

                                    new Pose(56.000, 88.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))

                    .build();

            IntakeFirstSet = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(56.000, 88.000),
                                    new Pose(45.684, 83.378),
                                    new Pose(24.000, 86.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            ToShoot5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(24.000, 86.000),

                                    new Pose(56.000, 110.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();
        }
    }

}
