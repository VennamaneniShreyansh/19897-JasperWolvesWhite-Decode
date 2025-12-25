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

import org.firstinspires.ftc.teamcode.helper.Data;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue Auto 9 Artifact Close", group = "Autonomous")
@Configurable // Panels
public class PedroAutonomousBlueClose extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(33.75, 135.5, Math.toRadians(180)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine
        Data.setAutoPose(follower.getPose());

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain ToShoot;
        public PathChain IntakeFirstSet;
        public PathChain ToShoot2;
        public PathChain IntakeSecondSet;
        public PathChain ToShoot3;
        public PathChain IntakeThirdSet;

        public Paths(Follower follower) {
            ToShoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(33.756, 135.220), new Pose(53.000, 90.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            IntakeFirstSet = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(53.000, 90.000),
                                    new Pose(45.500, 84.500),
                                    new Pose(19.500, 84.000)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            ToShoot2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(19.500, 84.000), new Pose(53.000, 90.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            IntakeSecondSet = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(53.000, 90.000),
                                    new Pose(52.000, 73.000),
                                    new Pose(60.000, 59.500),
                                    new Pose(19.000, 60.000)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            ToShoot3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(19.000, 60.000), new Pose(53.000, 90.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            IntakeThirdSet = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(53.000, 90.000),
                                    new Pose(61.500, 30.500),
                                    new Pose(44.500, 35.500),
                                    new Pose(17.500, 36.000)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                // Start first path: ToShoot
                if (!follower.isBusy()) {
                    follower.followPath(paths.ToShoot);
                    pathState = 1; // now wait for it to finish
                }
                break;

            case 1:
                // Wait for ToShoot to complete, then go intake first set
                if (!follower.isBusy()) {
                    follower.followPath(paths.IntakeFirstSet);
                    pathState = 2;
                }
                break;

            case 2:
                // Wait for IntakeFirstSet, then go back to shoot
                if (!follower.isBusy()) {
                    follower.followPath(paths.ToShoot2);
                    pathState = 3;
                }
                break;

            case 3:
                // Wait for ToShoot2, then intake second set
                if (!follower.isBusy()) {
                    follower.followPath(paths.IntakeSecondSet);
                    pathState = 4;
                }
                break;

            case 4:
                // Wait for IntakeSecondSet, then go back to shoot
                if (!follower.isBusy()) {
                    follower.followPath(paths.ToShoot3);
                    pathState = 5;
                }
                break;

            case 5:
                // Wait for ToShoot3, then intake third set
                if (!follower.isBusy()) {
                    follower.followPath(paths.IntakeThirdSet);
                    pathState = 6;
                }
                break;

            case 6:
                Data.setAutoPose(follower.getPose());
//                Data.turretYaw = follower.getPose().getHeading();
                break;

            default:
                // Safety fallback
                pathState = 6;
                break;
        }

        return pathState;
    }
}
