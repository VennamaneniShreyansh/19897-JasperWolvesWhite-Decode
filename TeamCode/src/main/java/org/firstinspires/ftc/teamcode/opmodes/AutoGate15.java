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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.helper.Alliance;
import org.firstinspires.ftc.teamcode.helper.Data;
import org.firstinspires.ftc.teamcode.helper.RapidFire;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
//import org.firstinspires.ftc.teamcode.pedroPathing.Drawing; // Path visualization


@Autonomous(name = "Blue Auto RapidFire 15 Gate", group = "Autonomous")
@Configurable
@Config
public class AutoGate15 extends OpMode {

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
        // 1. Convert position
        double ftcY = -72 + pedroX;   // FTC +Y → Pedro +X
        double ftcX = 72 - pedroY;   // FTC +X (down) → Pedro -Y (up)

        // 2. Convert heading
        double ftcHeading = pedroHeading + Math.PI / 2;

        // 3. Normalize to 0 → 2π
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
        paths = new Paths(follower, robot);
        threeBallShooter = new RapidFire(robot.intake, robot.outtake);

        Data.setAutoPose(follower.getPose());

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
        pathState = 0;
    }

    @Override
    public void start() {
        robot.hood.set(.5, .5);
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

//        Tuning.draw();
//        Tuning.drawOnlyCurrent();
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
                robot.turret.setTurretTarget(-130);
                robot.shootHigh();
                if (!follower.isBusy()) {
                    robot.gate.closeGate();
                    follower.followPath(paths.ToShoot);
                    robot.gate.openGate();
                    pathState = 1;
                    settleStartTime = 0;
                }
                break;

            case 1: // Shoot
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
//                            robot.gate.closeGate();
                            robot.intakeIn();
                            follower.setMaxPower(1);
                            follower.followPath(paths.IntakeSecondSet);
                            pathState = 2;
                        }
                    }
                }
                break;

            case 2: // From intake to Shoot 2
                if (!follower.isBusy()) {
                    if (stopCheckTime == 0) {
                        stopCheckTime = System.currentTimeMillis();
                    }

                    if (System.currentTimeMillis() - stopCheckTime >= 50) {
                        follower.setMaxPower(1);
                        follower.followPath(paths.ToShoot2);
                        pathState = 3;
                        stopCheckTime = 0;
                        settleStartTime = 0;
                    }
                }
                break;

            case 3: // Shoot 2nd 3
                if (!follower.isBusy()) {
//                    robot.gate.openGate();

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
//                            robot.gate.closeGate();
                            robot.intakeIn();
                            follower.setMaxPower(1);
                            follower.followPath(paths.OpenGateIntake);
                            pathState = 4;
                        }
                    }
                }
                break;

            case 4: // Intake to shoot
                if (!follower.isBusy()) {
                    if (stopCheckTime == 0) {
                        stopCheckTime = System.currentTimeMillis();
                    }

                    if (System.currentTimeMillis() - stopCheckTime >= 1000) {
                        follower.setMaxPower(1);
//                        robot.intakeOff();
                        follower.followPath(paths.ToShoot3);
                        pathState = 5;
                        stopCheckTime = 0;
                        settleStartTime = 0;
                    }
                }
                break;

            case 5: // Shoot last 3
                if (!follower.isBusy()) {
//                    robot.gate.openGate();

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
//                            robot.gate.closeGate();
                            robot.intakeIn();
                            follower.setMaxPower(1);
                            follower.followPath(paths.IntakeFirstSet);
                            pathState = 6;
                        }
                    }
                }
                break;
            case 6: // Go to Shoot 4
                if (!follower.isBusy()) {
                    if (stopCheckTime == 0) {
                        stopCheckTime = System.currentTimeMillis();
                    }

                    if (System.currentTimeMillis() - stopCheckTime >= 50) {
//                        robot.intakeOff();
//                        robot.turret.setTurretTarget(-130);
                        follower.followPath(paths.ToShoot4);
                        settleStartTime = 0;
                        pathState = 7;
                    }
                }
                break;

            case 7: // Shoot last 3
                if (!follower.isBusy()) {
//                    robot.gate.openGate();

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
                            robot.intakeIn();
                            follower.setMaxPower(1);
                            follower.followPath(paths.IntakeThirdSet);
                            pathState = 8;
                        }
                    }
                }
                break;

            case 8: // Shoot 4th set
                if (!follower.isBusy()) {
                    if (stopCheckTime == 0) {
                        stopCheckTime = System.currentTimeMillis();
                    }

                    if (System.currentTimeMillis() - stopCheckTime >= 50) {
//                        robot.intakeOff();
                        robot.turret.setTurretTarget(-70);
                        follower.followPath(paths.ToShoot5);
                        settleStartTime = 0;
                        pathState = 9;
                    }
                }
                break;
            case 9:
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
                            pathState = 10;
                        }
                    }
                }
                break;
            case 10: // Leave launch pad
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
        public PathChain OpenGateIntake;
        public PathChain ToShoot3;
        public PathChain IntakeFirstSet;
        public PathChain ToShoot4;
        public PathChain IntakeThirdSet;
        public PathChain ToShoot5;
        private final Robot robot;  // non-static

        public Paths(Follower follower, Robot robot) {  // take robot param
            this.robot = robot;
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
                                    new Pose(19.000, 58.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .addParametricCallback(0.3, robot.gate::closeGate)
                    .build();

            ToShoot2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(19.000, 58.000),

                                    new Pose(56.000, 88.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .addParametricCallback(0.4, robot::intakeOff)
                    .addParametricCallback(0.67, robot.gate::openGate)
                    .build();

            OpenGateIntake = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(56.000, 88.000),
                                    new Pose(38.200, 57.900),
                                    new Pose(2.884, 64.441),
                                    new Pose(11.200, 54.737)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                    .build();

            ToShoot3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(11.200, 53.000),
                                    new Pose(39.000, 72.600),
                                    new Pose(56.000, 88.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(180))
                    .addParametricCallback(0.4, robot::intakeOff)
                    .addParametricCallback(0.67, robot.gate::openGate)
                    .build();

            IntakeFirstSet = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(56.000, 88.000),
                                    new Pose(45.600, 81.370),
                                    new Pose(22.000, 84.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .addParametricCallback(0.1, robot.gate::closeGate)
                    .build();

            ToShoot4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(22.000, 84.000),

                                    new Pose(56.000, 88.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .addParametricCallback(0.4, robot::intakeOff)
                    .addParametricCallback(0.67, robot.gate::openGate)
                    .build();

            IntakeThirdSet = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(56.000, 110.000),
                                    new Pose(61.500, 30.500),
                                    new Pose(44.500, 35.500),
                                    new Pose(19.000, 36.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .addParametricCallback(0.35, robot.gate::closeGate)
                    .build();

            ToShoot5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(19.000, 36.000),

                                    new Pose(56.000, 110.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .addParametricCallback(0.4, robot::intakeOff)
                    .addParametricCallback(0.67, robot.gate::openGate)
                    .build();
        }
    }


}