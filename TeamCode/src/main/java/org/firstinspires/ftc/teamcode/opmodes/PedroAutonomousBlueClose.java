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
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.helper.Alliance;
import org.firstinspires.ftc.teamcode.helper.Data;
import org.firstinspires.ftc.teamcode.helper.ThreeBallShooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous(name = "Blue Auto 9 Artifact Close", group = "Autonomous")
@Configurable // Panels
public class PedroAutonomousBlueClose extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private ThreeBallShooter threeBallShooter;
    private Robot robot;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(33.75, 135.5, Math.toRadians(180)));

        paths = new Paths(follower);

        robot = new Robot(hardwareMap, Alliance.BLUE);
        threeBallShooter = new ThreeBallShooter(robot.intake, robot.outtake);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        pathState = autonomousPathUpdate();
        Data.setAutoPose(follower.getPose());
        robot.periodic();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public int autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                // Close Gate + Start Outtake, then move to shoot position
                if (!follower.isBusy()) {
                    robot.gate.closeGate();
                    robot.shootLow();
                    follower.followPath(paths.ToShoot);
                    pathState = 1;
                }
                break;

            case 1:
                // SHOOT 3 BALLS
                if (!follower.isBusy()) {
                    if (!threeBallShooter.isActive() && !threeBallShooter.isDone()) {
                        threeBallShooter.start();
                    }
                    if (threeBallShooter.isActive()) {
                        threeBallShooter.update();
                    }
                    if (threeBallShooter.isDone()) {
                        // After shooting, close gate + start intake to collect next set
                        robot.gate.closeGate();
                        robot.intakeIn();
                        follower.followPath(paths.IntakeFirstSet);
                        pathState = 2;
                    }
                } else {
                    threeBallShooter.update();
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    // Stop intake + open gate to prepare for next shooting
                    robot.intakeOff();
                    robot.gate.openGate();
                    follower.followPath(paths.ToShoot2);
                    pathState = 3;
                }
                break;

            case 3:
                // SHOOT 3 BALLS AGAIN
                if (!follower.isBusy()) {
                    if (!threeBallShooter.isActive() && !threeBallShooter.isDone()) {
                        threeBallShooter.start();
                    }
                    if (threeBallShooter.isActive()) {
                        threeBallShooter.update();
                    }
                    if (threeBallShooter.isDone()) {
                        // Close gate + start intake for next pickup
                        robot.gate.closeGate();
                        robot.intakeIn();
                        follower.followPath(paths.IntakeSecondSet);
                        pathState = 4;
                    }
                } else {
                    threeBallShooter.update();
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    // Stop intake + open gate, prepare to shoot again
                    robot.intakeOff();
                    robot.gate.openGate();
                    follower.followPath(paths.ToShoot3);
                    pathState = 5;
                }
                break;

            case 5:
                // FINAL SHOOT
                if (!follower.isBusy()) {
                    if (!threeBallShooter.isActive() && !threeBallShooter.isDone()) {
                        threeBallShooter.start();
                    }
                    if (threeBallShooter.isActive()) {
                        threeBallShooter.update();
                    }
                    if (threeBallShooter.isDone()) {
                        // Close gate + stop outtake + start intake to clear final set
                        robot.gate.closeGate();
                        robot.stopShooter();
                        robot.intakeIn();
                        follower.followPath(paths.IntakeThirdSet);
                        pathState = 6;
                    }
                } else {
                    threeBallShooter.update();
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    // Stop intake after final collection
                    robot.intakeOff();
                    Data.setAutoPose(follower.getPose());
                }
                break;

            default:
                pathState = 6;
                break;
        }
        return pathState;
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
}
