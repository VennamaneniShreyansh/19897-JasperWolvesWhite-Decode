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
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Aura Auton", group = "Autonomous")
@Configurable
@Disabled
public class FullAutonAura extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState = 0;
    private Paths paths;
    private long stateStartTime;
    private boolean pathStarted = false;

    // Intake & Outtake motors
    private DcMotor intakeMotor;
    private DcMotor outtakeLeft;
    private DcMotor outtakeRight;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower);

        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        outtakeLeft = hardwareMap.get(DcMotor.class, "outtakeLeft");
        outtakeRight = hardwareMap.get(DcMotor.class, "outtakeRight");

        // Reverse one outtake motor
        outtakeRight.setDirection(DcMotor.Direction.REVERSE);

        stopAllMotors();
        stateStartTime = System.currentTimeMillis();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        pathState = autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    private int autonomousPathUpdate() {
        long elapsed = System.currentTimeMillis() - stateStartTime;

        switch (pathState) {
            case 0:
                // Start, turn on outtake
                setOuttakePower(1);
                if (elapsed > 5000) {
                    // After 5sec, start intake to push balls up
                    setIntakePower(1);
                    if (elapsed > 8000) { // run intake for 3sec
                        stopAllMotors();
                        nextState();
                    }
                }
                break;

            case 1:
                // Path 1
                if (!pathStarted) {
                    follower.followPath(paths.Path1);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    nextState();
                }
                break;

            case 2:
                // Path 2: intake ON (collect balls)
                setIntakePower(1);
                if (!pathStarted) {
                    follower.followPath(paths.Path2);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    stopAllMotors();
                    nextState();
                }
                break;

            case 3:
                // Path 3: outtake ON (score)
                setOuttakePower(1);
                if (!pathStarted) {
                    follower.followPath(paths.Path3);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    if (elapsed > 3000) { // wait 3s at end
                        setIntakePower(1); // push balls into outtake
                        if (elapsed > 6000) { // run intake+outtake for 3s
                            stopAllMotors();
                            nextState();
                        }
                    }
                }
                break;

            case 4:
                // Move through path 4 (no intake/outtake)
                if (!pathStarted) {
                    follower.followPath(paths.Path4);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    nextState();
                }
                break;

            case 5:
                // Path 5: intake ON while driving
                setIntakePower(1);
                if (!pathStarted) {
                    follower.followPath(paths.Path5);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    stopAllMotors();
                    nextState();
                }
                break;

            case 6:
                // Path 6: outtake ON (scoring again)
                setOuttakePower(1);
                if (!pathStarted) {
                    follower.followPath(paths.Path6);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    if (elapsed > 3000) { // wait 3s
                        setIntakePower(1); // feed into outtake
                        if (elapsed > 6000) { // run both for 3s
                            stopAllMotors();
                            nextState();
                        }
                    }
                }
                break;

            case 7:
                // Path 7: final movement, all off
                if (!pathStarted) {
                    follower.followPath(paths.Path7);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    nextState();
                }
                break;

            case 8:
                // Done
                stopAllMotors();
                break;
        }

        return pathState;
    }

    private void nextState() {
        pathState++;
        pathStarted = false;
        stateStartTime = System.currentTimeMillis();
    }

    private void setIntakePower(double p) {
        intakeMotor.setPower(p);
    }

    private void setOuttakePower(double p) {
        outtakeLeft.setPower(p);
        outtakeRight.setPower(p);
    }

    private void stopAllMotors() {
        intakeMotor.setPower(0);
        outtakeLeft.setPower(0);
        outtakeRight.setPower(0);
    }

    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(75.000, 76.000),
                            new Pose(61.000, 83.900),
                            new Pose(45.000, 84.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(180))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(45.000, 84.000), new Pose(17.000, 84.000)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(17.000, 84.000), new Pose(75.000, 76.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(315))
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(75.000, 76.000),
                            new Pose(63.000, 63.000),
                            new Pose(45.000, 60.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(180))
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(45.000, 60.000), new Pose(11.000, 60.000)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(11.000, 60.000), new Pose(75.000, 76.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(315))
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(75.000, 76.000), new Pose(60.000, 36.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(180))
                    .build();
        }
    }
}
