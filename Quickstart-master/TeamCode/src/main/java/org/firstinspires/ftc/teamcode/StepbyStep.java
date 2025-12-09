package org.firstinspires.ftc.teamcode;
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

@Autonomous(name = "Frickity Fricking Frick", group = "Autonomous")
@Configurable // Panels
public class StepbyStep extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)// Paths defined in the Paths class
    private DcMotorEx outtakeLeft, outtakeRight;
    private DcMotor intakeMotor;
    private Servo servo;
    private static final double TICKS_PER_REV = 28;
    private static final double TARGET_RPM_LOW = 2650;
    private long shootTimer = 0;
    private int shootStage = 0;
    private int ballsShot = 0;
    private static final long BALL_TIMEOUT_MS = 2000;
    private double stateTimer = 0;


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

        // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {


        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }


    private void startIntake() {
        double INTAKE_POWER = .95;
        intakeMotor.setPower(INTAKE_POWER);
    }

    private void stopIntake() {
        intakeMotor.setPower(0);
    }

    private void openServo() {
        double OPEN_SERVO = 0.15;
        servo.setPosition(OPEN_SERVO);
    }
    private void closeServo() {
        double CLOSE_SERVO = 0.0;
        servo.setPosition(CLOSE_SERVO);
    }


    private void shootBalls() {
        long now = System.currentTimeMillis();

        switch (shootStage) {

            // Start intake for next ball
            case 0:
                if (ballsShot >= 3) {
                    stopIntake();     // safety
                    shootStage = 999; // finished
                    break;
                }

                startIntake();
                shootTimer = now;
                shootStage = 1;
                break;

            // Run intake for 1750 ms
            case 1:
                if (now - shootTimer >= 1750) {
                    stopIntake();
                    shootTimer = now;
                    shootStage = 2;
                }
                break;

            // Pause for 500 ms
            case 2:
                if (now - shootTimer >= 500) {
                    ballsShot++;
                    shootStage = 0; // repeat for next ball
                }
                break;

            // Optional: finished state
            case 999:
                // Do nothing—shooting complete
                break;
        }
    }

}