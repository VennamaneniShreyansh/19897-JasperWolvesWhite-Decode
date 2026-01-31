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
import org.firstinspires.ftc.teamcode.helper.ThreeBallShooterClose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Tuning;  // Has draw() methods
//import org.firstinspires.ftc.teamcode.pedroPathing.Drawing; // Path visualization
import org.firstinspires.ftc.teamcode.pedroPathing.Tuning.*;


@Autonomous(name = "Spare Blue Auto", group = "Autonomous")
@Configurable
@Config
public class SpareBlueGate extends OpMode {

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
        paths = new Paths(follower);
        robot = new Robot(hardwareMap, Alliance.BLUE, false);
        threeBallShooter = new RapidFire(robot.intake, robot.outtake);

//        robot.outtake.periodic();
        Data.setAutoPose(follower.getPose());

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
        pathState = 0;
    }

    @Override
    public void start() {
        robot.hood.set(.1, .9);
        robot.outtake.periodic();
    }

    @Override
    public void loop() {
        follower.update();

//        robot.outtake.periodic();
        robot.autoPeriodic();
        threeBallShooter.update();

        Data.setAutoPose(follower.getPose());

        pathState = autonomousPathUpdate();
        robot.shootLow();

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
        panelsTelemetry.debug("Outtake Target", robot.outtake.targetRPM);
        panelsTelemetry.debug("Shooter enabled", robot.outtake.isEnabled());
        panelsTelemetry.debug("Shooter target", robot.outtake.targetRPM);

        panelsTelemetry.debug("At Target", robot.outtake.atTarget());
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.update(telemetry);
    }

    public int autonomousPathUpdate() {
        switch (pathState) {

            case 0: // Go to First Shoot
                robot.turret.setTurretTarget(-120);
                robot.shootLow();
                if (!follower.isBusy()) {
                    robot.gate.closeGate();
                    follower.followPath(paths.ToShoot);
                    robot.gate.openGate();
                    pathState = 1;
                    settleStartTime = 0;
                }
                break;

            case 1: // Shoot 3
                if (!follower.isBusy()) {

                    if (settleStartTime == 0) {
                        settleStartTime = System.currentTimeMillis();
                    }

                    if (System.currentTimeMillis() - settleStartTime >= 1000) {

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
                            follower.followPath(paths.IntakeFirstSet);
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

                    if (System.currentTimeMillis() - stopCheckTime >= 400) {
                        robot.intakeOff();
                        follower.setMaxPower(.8);
                        follower.followPath(paths.OpenGate);
                        pathState = 10;
                        stopCheckTime = 0;
                        settleStartTime = 0;
                    }
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    if (stopCheckTime == 0) {
                        stopCheckTime = System.currentTimeMillis();
                    }

                    if (System.currentTimeMillis() - stopCheckTime >= 1000) {
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
                    robot.gate.openGate();

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
                            follower.followPath(paths.IntakeSecondSet);
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

                    if (System.currentTimeMillis() - stopCheckTime >= 300) {
                        follower.setMaxPower(1);
                        robot.intakeOff();
                        follower.followPath(paths.ToShoot3);
                        pathState = 5;
                        stopCheckTime = 0;
                        settleStartTime = 0;
                    }
                }
                break;

            case 5: // Shoot last 3
                if (!follower.isBusy()) {
                    robot.gate.openGate();

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
                            follower.followPath(paths.IntakeThirdSet);
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

                    if (System.currentTimeMillis() - stopCheckTime >= 300) {
                        robot.intakeOff();
                        robot.turret.setTurretTarget(-113);
                        follower.followPath(paths.ToShoot4);
                        settleStartTime = 0;
                        pathState = 7;
                    }
                }
                break;

            case 7: // Shoot 4th set
                if (!follower.isBusy()) {
                    robot.gate.openGate();

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
                            follower.followPath(paths.LeaveLaunchPad);
                            pathState = 8;
                        }
                    }
                }
                break;
            case 8: // Leave launch pad
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
        public PathChain ToShoot, IntakeFirstSet, ToShoot2, IntakeSecondSet, ToShoot3,
                IntakeThirdSet, ToShoot4, LeaveLaunchPad, OpenGate;

        public Paths(Follower follower) {
            ToShoot = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(33.756, 135.220), new Pose(41.000, 105.000)))
                    .setConstantHeadingInterpolation(Math.toRadians(180)).build();

            IntakeFirstSet = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(53.000, 90.000), new Pose(51.750, 81.370), new Pose(24.500, 84.000)))
                    .setConstantHeadingInterpolation(Math.toRadians(180)).build();

            OpenGate = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(24.500, 84.000),
                                    new Pose(26.660, 76.616),
                                    new Pose(20, 74)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(90))

                    .build();

            ToShoot2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(25.000, 84.000), new Pose(41.000, 101.000)))
                    .setConstantHeadingInterpolation(Math.toRadians(180)).build();

            IntakeSecondSet = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(53.000, 90.000), new Pose(52.000, 73.000), new Pose(60.000, 59.500), new Pose(19.000, 58.000)))
                    .setConstantHeadingInterpolation(Math.toRadians(180)).build();

            ToShoot3 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(22.000, 58.000), new Pose(39.000, 72.600), new Pose(41.000, 101.000)))
                    .setConstantHeadingInterpolation(Math.toRadians(180)).build();

            IntakeThirdSet = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(53.000, 90.000), new Pose(61.500, 30.500), new Pose(44.500, 35.500), new Pose(19.00, 36.000)))
                    .setConstantHeadingInterpolation(Math.toRadians(180)).build();

            ToShoot4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(17.500, 36.000), new Pose(41.000, 101.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            LeaveLaunchPad = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(53.000, 90.000), new Pose(39.000, 77.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();


        }
    }
}