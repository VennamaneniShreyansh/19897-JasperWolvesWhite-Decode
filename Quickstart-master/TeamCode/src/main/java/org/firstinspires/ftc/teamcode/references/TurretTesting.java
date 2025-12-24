//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.IMU;
//import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//
//
//@TeleOp(name = "Turret Auto Aim SIMPLE + SEARCH", group = "TeleOp")
//public class TurretTesting extends OpMode {
//    private IMU imu;
//    private DcMotorEx turret;
//    private Limelight3A limelight;
////    private static
//    private static final double kP = 0.02;
//    private static final double MAX_POWER = 0.8;
//    private static final double TX_DEADBAND = 1.0;
//
////    private static final double SEARCH_POWER = 0.34;
//    private static final double SEARCH_ANGLE_DEG = 10.0;
//    private static final double TICKS_PER_MOTOR_REV = 8192.0;
//    private static final double MOTOR_GEAR_TEETH = 31.0;
//    private static final double TURRET_GEAR_TEETH = 100.0;
//
//    private static final double TICKS_PER_TURRET_REV =
//            TICKS_PER_MOTOR_REV * (TURRET_GEAR_TEETH / MOTOR_GEAR_TEETH);
//
//    private static final double TICKS_PER_DEGREE =
//            TICKS_PER_TURRET_REV / 360.0;
//
//    private double getTurretAngleDeg() {
//        return turret.getCurrentPosition() / TICKS_PER_DEGREE;
//    }
//
//    private enum State {
//        SEARCH_LEFT,
//        SEARCH_RIGHT,
//        TRACKING
//    }
//    private State state = State.SEARCH_LEFT;
//
//    @Override
//    public void init() {
//        turret = hardwareMap.get(DcMotorEx.class, "turret");
//        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.pipelineSwitch(0);
//
//        imu = hardwareMap.get(IMU.class, "imu");
//
//        IMU.Parameters params = new IMU.Parameters(
//                new com.qualcomm.hardware.rev.RevHubOrientationOnRobot(
//                        com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
//                        com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection.UP
//                )
//        );
//
//        imu.initialize(params);
//
//    }
//
//    private boolean robotTurningLeft() {
//        AngularVelocity av = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
//        return av.zRotationRate > 3; // small deadband to ignore noise
//    }
//
//    private boolean robotTurningRight() {
//        AngularVelocity av = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
//        return av.zRotationRate < -3;
//    }
//
//
//    @Override
//    public void start() {
//        limelight.start();
//    }
//
//    @Override
//    public void loop() {
//        LLResult result = limelight.getLatestResult();
//        boolean hasTarget = result != null && result.isValid();
//
//        double turretAngle = getTurretAngleDeg();
//        double power = 0;
//
//        if (hasTarget) {
//            state = State.TRACKING;
//
//            double tx = result.getTx();
//
//            // where turret is now + limelight error
//            double targetAngle = turretAngle + tx;
//
//            rotateTurretTo(targetAngle);
//
//            telemetry.addData("State", "TRACKING");
//            telemetry.addData("tx (deg)", tx);
//            telemetry.addData("Target Angle", targetAngle);
//        }
//
//        else {
//            if (state == State.TRACKING) {
//                if (robotTurningLeft()) {
//                    state = State.SEARCH_RIGHT; // counter-rotate turret
//                } else if (robotTurningRight()) {
//                    state = State.SEARCH_LEFT;
//                } else {
//                    state = State.SEARCH_LEFT; // default if robot is still
//                }
//            }
//            // Remove this if acting wierd
////            if (robotTurningLeft()) {
////                state = State.SEARCH_RIGHT;
////            } else if (robotTurningRight()) {
////                state = State.SEARCH_LEFT;
////            }
//
//            switch (state) {
//
//                case SEARCH_LEFT:
//                    rotateTurretTo(-7);
//                    if (turretAngle <= -7 + 1.0) {
//                        state = State.SEARCH_RIGHT;
//                    }
//                    break;
//
//                case SEARCH_RIGHT:
//                    rotateTurretTo(SEARCH_ANGLE_DEG);
//                    if (turretAngle >= SEARCH_ANGLE_DEG - 1.0) {
//                        state = State.SEARCH_LEFT;
//                    }
//                    break;
//            }
//
//            telemetry.addData("State", state);
//        }
//        AngularVelocity av = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
//        telemetry.addData("Yaw Rate (deg/s)", av.zRotationRate);
//        telemetry.addData("Encoder Ticks", turret.getCurrentPosition());
//        telemetry.addData("Turret Angle (deg)", turretAngle);
//        telemetry.update();
//    }
//
//    private void rotateTurretTo(double targetDeg) {
//        double error = targetDeg - getTurretAngleDeg();
//
//        if (Math.abs(error) < TX_DEADBAND) {
//            turret.setPower(0);
//            return;
//        }
//
//        double power = kP * error;
//        turret.setPower(clamp(power, -MAX_POWER, MAX_POWER));
//    }
//
//
//    private double clamp(double v, double min, double max) {
//        return Math.max(min, Math.min(max, v));
//    }
//}



package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Turret Auto Aim SIMPLE + SEARCH", group = "TeleOp")
public class TurretTesting extends OpMode {
    private DcMotorEx turret;
    private Limelight3A limelight;

    private static final double kP = 0.011;
    private static final double MAX_POWER = 0.8;
    private static final double TX_DEADBAND = 1.0;

    private static final double SEARCH_POWER = 0.24;
    private static final double SEARCH_ANGLE_DEG = 8.0;
    private static final double TICKS_PER_REV = 8192.0;
    private static final double GEAR_RATIO = (double) 100 /31;

    private enum State {
        SEARCH_LEFT,
        SEARCH_RIGHT,
        TRACKING
    }
    private State state = State.SEARCH_LEFT;

    @Override
    public void init() {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();
        boolean hasTarget = result != null && result.isValid();

        double turretAngle = getTurretAngleDeg();
        double power = 0;

        // Tracking
        if (hasTarget) {
            state = State.TRACKING;

            double tx = result.getTx();
            power = -kP * tx;

            if (Math.abs(tx) < TX_DEADBAND) {
                power = 0;
            }

            power = clamp(power, -MAX_POWER, MAX_POWER);

            telemetry.addData("State", "TRACKING");
            telemetry.addData("tx", tx);
        }

        // Search
        else {
            if (state == State.TRACKING) {
                // if lose target, search left
                state = State.SEARCH_LEFT;
            }
            switch (state) {

                case SEARCH_LEFT:
                    power = -SEARCH_POWER;
                    if (turretAngle <= -SEARCH_ANGLE_DEG) {
                        state = State.SEARCH_RIGHT;
                    }
                    break;

                case SEARCH_RIGHT:
                    power = SEARCH_POWER;
                    if (turretAngle >= SEARCH_ANGLE_DEG) {
                        state = State.SEARCH_LEFT;
                    }
                    break;
            }

            telemetry.addData("State", state);
        }

        turret.setPower(power);

        telemetry.addData("Turret Angle", turretAngle);
        telemetry.addData("Power", power);
        telemetry.update();
    }
    private double getTurretAngleDeg() {
        return (turret.getCurrentPosition() / TICKS_PER_REV) * 360.0;
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}


//make this a proper 360 degrees so i don't have to guess numbers, so print me ticks and then the desgrees, so we know what makes it 360 fully