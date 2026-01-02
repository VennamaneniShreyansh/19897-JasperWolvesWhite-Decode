package org.firstinspires.ftc.teamcode.opmodes;

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
import org.firstinspires.ftc.teamcode.helper.Alliance;
import org.firstinspires.ftc.teamcode.helper.Data;
import org.firstinspires.ftc.teamcode.helper.ThreeBallShooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous(name = "BA Close 9 Artifact", group = "Autonomous")
@Configurable
public class PedroAutonomousBlueClose extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;
    private ThreeBallShooter threeBallShooter;
    private Robot robot;
    private long shootStartTime = 0;
    private long settleStartTime = 0;
    private long stopCheckTime = 0;
    private static final long SETTLE_MS = 250;


    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(33.75, 135.5, Math.toRadians(180)));
        paths = new Paths(follower);
        robot = new Robot(hardwareMap, Alliance.BLUE);
        threeBallShooter = new ThreeBallShooter(robot.intake, robot.outtake);

        robot.outtake.periodic();
        Data.setAutoPose(follower.getPose());

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
        pathState = 0;
    }

    @Override
    public void start() {
        robot.hood.set(.05, .95);
        robot.outtake.periodic();
    }

    @Override
    public void loop() {
        follower.update();

        robot.outtake.periodic();
        robot.autoPeriodic();
        threeBallShooter.update();

        pathState = autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("Shooter Stage", threeBallShooter.stage);
        panelsTelemetry.debug("Outtake RPM L", robot.outtake.getRPMLeft());
        panelsTelemetry.debug("Outtake RPM R", robot.outtake.getRPMRight());
        panelsTelemetry.debug("Outtake Target", robot.outtake.targetRPM);
        panelsTelemetry.debug("At Target", robot.outtake.atTarget());
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.update(telemetry);
    }

    public int autonomousPathUpdate() {
        switch (pathState) {

            case 0: // Go to First Shoot
                robot.turret.setTurretTarget(-120);
                robot.outtake.shootLow();
                //robot.turret.manual(0.7);
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
                            follower.setMaxPowerScaling(.75);
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

                    if (System.currentTimeMillis() - stopCheckTime >= 300) {
                        robot.intakeOff();
                        follower.followPath(paths.ToShoot2);
                        follower.setMaxPowerScaling(1);
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
                            follower.followPath(paths.IntakeSecondSet);
                            stopCheckTime = System.currentTimeMillis();
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

                    if (System.currentTimeMillis() - stopCheckTime >= 200) {
                        robot.intakeOff();
                        follower.followPath(paths.ToShoot3);
                        follower.setMaxPowerScaling(1);
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
                            robot.stopShooter();
                            robot.intakeIn();
                            follower.followPath(paths.IntakeThirdSet);
                            pathState = 6;
                        }
                    }
                }
                break;

            case 6: // End
                if (!follower.isBusy()) {
                    if (stopCheckTime == 0) {
                        stopCheckTime = System.currentTimeMillis();
                    }

                    if (System.currentTimeMillis() - stopCheckTime >= 200) {
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
        public PathChain ToShoot, IntakeFirstSet, ToShoot2, IntakeSecondSet, ToShoot3, IntakeThirdSet;

        public Paths(Follower follower) {
            ToShoot = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(33.756, 135.220), new Pose(53.000, 90.000)))
                    .setConstantHeadingInterpolation(Math.toRadians(180)).build();

            IntakeFirstSet = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(53.000, 90.000), new Pose(45.500, 84.500), new Pose(25.000, 84.000)))
                    .setConstantHeadingInterpolation(Math.toRadians(180)).build();

            ToShoot2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(25.000, 84.000), new Pose(53.000, 90.000)))
                    .setConstantHeadingInterpolation(Math.toRadians(180)).build();

            IntakeSecondSet = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(53.000, 90.000), new Pose(52.000, 73.000), new Pose(60.000, 59.500), new Pose(22.000, 58.000)))
                    .setConstantHeadingInterpolation(Math.toRadians(180)).build();

            ToShoot3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(22.000, 58.000), new Pose(53.000, 90.000)))
                    .setConstantHeadingInterpolation(Math.toRadians(180)).build();

            IntakeThirdSet = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(53.000, 90.000), new Pose(61.500, 30.500), new Pose(44.500, 35.500), new Pose(25.00, 36.000)))
                    .setConstantHeadingInterpolation(Math.toRadians(180)).build();
        }
    }
}