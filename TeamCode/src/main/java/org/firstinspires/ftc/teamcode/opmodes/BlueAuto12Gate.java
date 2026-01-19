
package org.firstinspires.ftc.teamcode.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.helper.Alliance;
import org.firstinspires.ftc.teamcode.helper.Data;
import org.firstinspires.ftc.teamcode.helper.RapidFire;
import org.firstinspires.ftc.teamcode.helper.ThreeBallShooterClose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "Blue Auto 12 Gate", group = "Autonomous")
@Configurable // Panels
public class BlueAuto12Gate extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private RapidFire threeBallShooter;
    private Robot robot;
    private long shootStartTime = 0;
    private long settleStartTime = 0;
    private long stopCheckTime = 0;
    private static final long SETTLE_MS = 50;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(33.863, 135.233, Math.toRadians(180)));
        paths = new BlueAuto12Gate.Paths(follower);
        robot = new Robot(hardwareMap, Alliance.BLUE);
        threeBallShooter = new RapidFire(robot.intake, robot.outtake);

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

        Data.setAutoPose(follower.getPose());

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


    public static class Paths {
        public PathChain ToShoot;
        public PathChain IntakeFirstSet;
        public PathChain OpenGate;
        public PathChain ToShoot2;
        public PathChain IntakeSecondSet;
        public PathChain ToShoot3;
        public PathChain IntakeThirdSet;
        public PathChain ToShoot4;
        public PathChain LeaveLaunchPad;

        public Paths(Follower follower) {
            ToShoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(33.863, 135.233),

                                    new Pose(53.000, 90.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            IntakeFirstSet = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(53.000, 90.000),
                                    new Pose(45.500, 84.500),
                                    new Pose(19.500, 84.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            OpenGate = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(19.500, 84.000),
                                    new Pose(20.665, 77.659),
                                    new Pose(16.500, 72.500)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            ToShoot2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(16.500, 72.500),

                                    new Pose(53.000, 90.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            IntakeSecondSet = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(53.000, 90.000),
                                    new Pose(52.000, 73.000),
                                    new Pose(60.000, 59.500),
                                    new Pose(19.000, 60.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            ToShoot3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(19.000, 60.000),
                                    new Pose(33.000, 76.000),
                                    new Pose(53.000, 90.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            IntakeThirdSet = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(53.000, 90.000),
                                    new Pose(61.500, 30.500),
                                    new Pose(44.500, 35.500),
                                    new Pose(12.000, 36.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            ToShoot4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(12.000, 36.000),

                                    new Pose(53.000, 90.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            LeaveLaunchPad = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(53.000, 90.000),

                                    new Pose(39.000, 77.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();
        }
    }


    public int autonomousPathUpdate() {
        switch (pathState) {

            case 0: // Go to First Shoot
                robot.turret.setTurretTarget(-123);
                robot.outtake.shootLow();
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
                        pathState = 10;
                        stopCheckTime = 0;
                        settleStartTime = 0;
                    }
                }
                break;

            case 10: // To Gate
                if (!follower.isBusy()) {
                    if (stopCheckTime == 0) {
                        stopCheckTime = System.currentTimeMillis();
                    }

                    if (System.currentTimeMillis() - stopCheckTime >= 300) {
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

                    if (System.currentTimeMillis() - stopCheckTime >= 300) {
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
            case 6: // Go to Shoot 4
                if (!follower.isBusy()) {
                    robot.intakeOff();
                    robot.turret.setTurretTarget(-126);
                    robot.outtake.shootLow();
                    follower.followPath(paths.ToShoot4);
                    settleStartTime = 0;
                    pathState = 7;
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
}
    