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

import org.firstinspires.ftc.teamcode.helper.Alliance;
import org.firstinspires.ftc.teamcode.helper.Data;
import org.firstinspires.ftc.teamcode.helper.ThreeBallShooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous(name = "Blue Auto 9 Artifact Close", group = "Autonomous")
@Configurable
@Disabled
public class PA extends OpMode {

    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private Paths paths;
    private Robot robot;

    private ThreeBallShooter shooter;
    private int pathState = 0;
    private boolean shooterStarted = false;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(33.75, 135.5, Math.toRadians(180)));

        paths = new Paths(follower);

        robot = new Robot(hardwareMap, Alliance.BLUE);
        shooter = new ThreeBallShooter(robot.intake, robot.outtake);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        robot.turret.setTurretTarget(-150);
        robot.turret.automatic();
        robot.hood.set(.05, .95);

        follower.followPath(paths.ToShoot);
        pathState = 0;
    }

    @Override
    public void loop() {
        follower.update();
        shooter.update();   // 🔥 ALWAYS UPDATE SHOOTER
        robot.autoPeriodic();

        autonomousPathUpdate();

        panelsTelemetry.debug("State", pathState);
        panelsTelemetry.debug("Shooter Active", shooter.isActive());
        panelsTelemetry.debug("Shooter Done", shooter.isDone());
        panelsTelemetry.update(telemetry);
    }

    private void autonomousPathUpdate() {
        switch (pathState) {

            /* ---------- FIRST SHOOT ---------- */
            case 0:
                if (!follower.isBusy()) {
                    startShooterOnce();
                    if (shooter.isDone()) {
                        resetShooter();
                        robot.intakeIn();
                        follower.followPath(paths.IntakeFirstSet);
                        pathState = 1;
                    }
                }
                break;

            case 1:
                if (!follower.isBusy()) {
                    robot.intakeOff();
                    follower.followPath(paths.ToShoot2);
                    pathState = 2;
                }
                break;

            /* ---------- SECOND SHOOT ---------- */
            case 2:
                if (!follower.isBusy()) {
                    startShooterOnce();
                    if (shooter.isDone()) {
                        resetShooter();
                        robot.intakeIn();
                        follower.followPath(paths.IntakeSecondSet);
                        pathState = 3;
                    }
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    robot.intakeOff();
                    follower.followPath(paths.ToShoot3);
                    pathState = 4;
                }
                break;

            /* ---------- FINAL SHOOT ---------- */
            case 4:
                if (!follower.isBusy()) {
                    startShooterOnce();
                    if (shooter.isDone()) {
                        robot.stopShooter();
                        robot.turret.setTurretTarget(0);
                        Data.setAutoPose(follower.getPose());
                        pathState = 5;
                    }
                }
                break;
        }
    }

    /* ---------------- HELPERS ---------------- */

    private void startShooterOnce() {
        if (!shooterStarted) {
            robot.gate.closeGate();
            shooter.start();
            shooterStarted = true;
        }
    }

    private void resetShooter() {
        shooterStarted = false;
        shooter = new ThreeBallShooter(robot.intake, robot.outtake);
    }

    /* ---------------- PATHS ---------------- */

    public static class Paths {
        public PathChain ToShoot, IntakeFirstSet, ToShoot2, IntakeSecondSet, ToShoot3;

        public Paths(Follower follower) {

            ToShoot = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(33.75, 135.5),
                            new Pose(53.0, 90.0)))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            IntakeFirstSet = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(53.0, 90.0),
                            new Pose(45.5, 84.5),
                            new Pose(24.5, 84.0)))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            ToShoot2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(24.5, 84.0),
                            new Pose(53.0, 90.0)))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            IntakeSecondSet = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(53.0, 90.0),
                            new Pose(52.0, 73.0),
                            new Pose(19.0, 60.0)))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            ToShoot3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(19.0, 60.0),
                            new Pose(53.0, 90.0)))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();
        }
    }
}
