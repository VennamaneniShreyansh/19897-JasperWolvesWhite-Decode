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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "PP Near Blue Autonomous", group = "Autonomous")
@Configurable // Panels
public class PedroAutonNearBlue extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private DcMotorEx outtakeLeft, outtakeRight;
    private DcMotor intakeMotor;
    private Servo servo;
    private static final double TICKS_PER_REV = 28;
    private static final double TARGET_RPM_LOW = 2650;
//    private boolean outtakeOn = false;

    private double lastTargetTicks = 0;
//    private long stableStartTime = 0;

    @Override
    public void init() {
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        outtakeLeft = hardwareMap.get(DcMotorEx.class, "outtakeLeft");
        outtakeRight = hardwareMap.get(DcMotorEx.class, "outtakeRight");
        servo = hardwareMap.get(Servo.class, "servo");

        // PIDF
        double kP = 20, kI = 0, kD = 1.0;
        double kF = (32767.0 / (6000 * TICKS_PER_REV)) * 60.0;
        outtakeLeft.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        outtakeRight.setVelocityPIDFCoefficients(kP, kI, kD, kF);

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(21.46, 122.73, Math.toRadians(324)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }
    public void stopOuttake() {
//        outtakeOn = false;

        outtakeLeft.setVelocity(0);
        outtakeRight.setVelocity(0);
        lastTargetTicks = 0;
    }
//    private double rpm(DcMotorEx motor) {
//        return motor.getVelocity() * 60 / TICKS_PER_REV;
//    }
    private void startOuttake() {
//        outtakeOn = true;
        double targetTicks = TARGET_RPM_LOW * TICKS_PER_REV / 60.0;

        if (targetTicks != lastTargetTicks) {
            outtakeLeft.setVelocity(-targetTicks);
            outtakeRight.setVelocity(targetTicks);

            lastTargetTicks = targetTicks;
        }
//        double leftRPM  = Math.abs(rpm(outtakeLeft));
//        double rightRPM = Math.abs(rpm(outtakeRight));
    }
    private void startIntake() {
        double INTAKE_POWER = .95;
        intakeMotor.setPower(INTAKE_POWER);
    }
    private void stopIntake() {
        intakeMotor.setPower(0);
    }
    private void closeServo() {
        double CLOSE_SERVO_POS = 0;
        servo.setPosition(CLOSE_SERVO_POS);
    }
    private void openServo() {
        double OPEN_SERVO_POS = 0;
        servo.setPosition(OPEN_SERVO_POS);
    }
    private void shootBalls() {

    }

    public static class Paths {
        public PathChain ToShoot;
        public PathChain ToFirstSet;
        public PathChain IntakeFirstSet;
        public PathChain ToShoot2;
        public PathChain ToSecondSet;
        public PathChain IntakeSecondSet;
        public PathChain ToShoot3;
        public PathChain ToThirdSet;
        public PathChain IntakeSetThree;

        public Paths(Follower follower) {
            ToShoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(21.46, 122.73), new Pose(53.800, 90.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(324), Math.toRadians(315))
                    .build();

            ToFirstSet = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(53.800, 90.000),
                                    new Pose(55.415, 83.707),
                                    new Pose(45.000, 84.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(324), Math.toRadians(180))
                    .build();

            IntakeFirstSet = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(45.000, 84.000), new Pose(19.500, 84.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            ToShoot2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(19.500, 84.000), new Pose(53.800, 90.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(324))
                    .build();

            ToSecondSet = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(53.800, 90.000),
                                    new Pose(59.000, 71.500),
                                    new Pose(50.000, 60.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(324), Math.toRadians(180))
                    .build();

            IntakeSecondSet = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(50.000, 60.000), new Pose(24.000, 60.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            ToShoot3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(24.000, 60.000), new Pose(53.800, 90.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(324))
                    .build();

            ToThirdSet = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(53.800, 90.000), new Pose(50.000, 35.500))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(324), Math.toRadians(180))
                    .build();

            IntakeSetThree = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(50.000, 35.500), new Pose(15.000, 36.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }

    public int autonomousPathUpdate() {

        switch (pathState) {

            case 0:
                // Start outtake
                startOuttake();
                follower.followPath(paths.ToShoot);
                pathState = 1;
                break;

            case 1:
                if (!follower.isBusy()) {
                    // Fire the balls, Open servo, Intake on to push balls to outtake
                    shootBalls();
                    pathState = 2;
                }
                break;

            case 2:
                // Drive to first set, close servo, stop outtake and intake
                stopOuttake();
                closeServo();
                follower.followPath(paths.ToFirstSet);
                pathState = 3;
                break;

            case 3:
                if (!follower.isBusy()) {
                    // Intake on to intake the first set, keep servo closed
                    startIntake();
                    closeServo();
                    follower.followPath(paths.IntakeFirstSet);
                    pathState = 4;
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    // Turn off intake, Turn on outtake while going to shoot, keep servo closed
                    stopIntake();
                    closeServo();
                    startOuttake();
                    follower.followPath(paths.ToShoot2);
                    pathState = 5;
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    // Fire the balls, Open servo, Intake on to push balls to outtake
                    openServo();
                    shootBalls();
                    follower.followPath(paths.ToSecondSet);
                    // Close servo, Outtake off
                    stopOuttake();
                    closeServo();
                    pathState = 6;
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    // Turn intake on, keep servo closed
                    startIntake();
                    closeServo();
                    follower.followPath(paths.IntakeSecondSet);
                    pathState = 7;
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    // Stop intake, keep outtake on, keep servo closed
                    stopIntake();
                    closeServo();
                    startOuttake();
                    follower.followPath(paths.ToShoot3);
                    pathState = 8;
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    // Fire the balls, Open servo, Intake on to push balls to outtake
                    openServo();
                    shootBalls();
                    follower.followPath(paths.ToThirdSet);
                    // Close Servo, turn off intake and outtake
                    closeServo();
                    startOuttake();
                    pathState = 9;
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    // Keep servo closed, turn on intake
                    closeServo();
                    startIntake();
                    follower.followPath(paths.IntakeSetThree);
                    pathState = 10;
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    // Intake off
                    stopIntake();
                    pathState = 67;
                }
                break;

            default:
                // Stop moving and finish auto
                break;
        }

        return pathState;
    }
}
