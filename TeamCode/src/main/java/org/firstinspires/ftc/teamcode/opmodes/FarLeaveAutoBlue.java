package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.helper.Alliance;
import org.firstinspires.ftc.teamcode.helper.Data;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous(name = "Far Leave Auto Blue", group = "Autonomous")
@Configurable
public class FarLeaveAutoBlue extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;
    private Robot robot;

    private long settleStartTime = 0;
    private static final long SETTLE_MS = 75;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56.5, 8.5, Math.toRadians(180)));

        paths = new Paths(follower);

        robot = new Robot(hardwareMap, Alliance.BLUE);

        Data.setAutoPose(follower.getPose());

        pathState = 0;

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        robot.turret.setTurretTarget(0);
    }

    @Override
    public void loop() {
        follower.update();

        follower.setMaxPower(.5);

        pathState = autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public int autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                robot.turret.setTurretTarget(0);

                follower.followPath(paths.Path1);
                settleStartTime = 0;
                pathState = 1;
                break;

            case 1: // FOLLOW PATH
                if (!follower.isBusy()) {

                    if (settleStartTime == 0) {
                        settleStartTime = System.currentTimeMillis();
                    }

                    if (System.currentTimeMillis() - settleStartTime >= SETTLE_MS) {
                        pathState = 2;
                        settleStartTime = 0;
                    }
                }
                break;

            case 2:
                robot.gate.closeGate();
                robot.turret.setTurretTarget(0);

                Data.setAutoPose(follower.getPose());
                requestOpModeStop();
                break;
        }

        return pathState;
    }

    public static class Paths {
        public PathChain Path1;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(56.500, 8.500),
                                    new Pose(40.000, 8.500)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();
        }

    }
}
