//
//package org.firstinspires.ftc.teamcode.opmodes;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.telemetry.TelemetryManager;
//import com.bylazar.telemetry.PanelsTelemetry;
//
//import org.firstinspires.ftc.teamcode.helper.Alliance;
//import org.firstinspires.ftc.teamcode.helper.Data;
//import org.firstinspires.ftc.teamcode.helper.RapidFire;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//import org.firstinspires.ftc.teamcode.subsystems.Robot;
//
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.paths.PathChain;
//import com.pedropathing.geometry.Pose;
//
//@Autonomous(name = "Pedro 12 Gate Path", group = "Autonomous")
//@Configurable // Panels
//public class GateTest extends OpMode {
//    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
//    public Follower follower; // Pedro Pathing follower instance
//    private int pathState; // Current autonomous path state (state machine)
//    private BlueAuto12Gate.Paths paths; // Paths defined in the Paths class
//    private RapidFire threeBallShooter;
//    private Robot robot;
//    private long shootStartTime = 0;
//    private long settleStartTime = 0;
//    private long stopCheckTime = 0;
//    private static final long SETTLE_MS = 50;
//
//    @Override
//    public void init() {
//        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(new Pose(33.75, 135.5, Math.toRadians(180)));
//        paths = new BlueAuto12Gate.Paths(follower);
//        robot = new Robot(hardwareMap, Alliance.BLUE, false);
//        threeBallShooter = new RapidFire(robot.intake, robot.outtake);
//
//        robot.outtake.periodic();
//        Data.setAutoPose(follower.getPose());
//
//        panelsTelemetry.debug("Status", "Initialized");
//        panelsTelemetry.update(telemetry);
//        pathState = 0;
//    }
//
//    @Override
//    public void start() {
//        robot.hood.set(.23, .77);
//        robot.outtake.periodic();
//        follower.setMaxPower(.7);
//    }
//
//    @Override
//    public void loop() {
//        follower.update();
//
//        robot.outtake.periodic();
//        robot.autoPeriodic();
//        threeBallShooter.update();
//
//        Data.setAutoPose(follower.getPose());
//
//        pathState = autonomousPathUpdate();
//
//        panelsTelemetry.debug("Path State", pathState);
//        panelsTelemetry.debug("Shooter Stage", threeBallShooter.stage);
//        panelsTelemetry.debug("Outtake RPM L", robot.outtake.getRPMLeft());
//        panelsTelemetry.debug("Outtake RPM R", robot.outtake.getRPMRight());
//        panelsTelemetry.debug("Outtake Target", robot.outtake.targetRPM);
//        panelsTelemetry.debug("At Target", robot.outtake.atTarget());
//        panelsTelemetry.debug("X", follower.getPose().getX());
//        panelsTelemetry.debug("Y", follower.getPose().getY());
//        panelsTelemetry.update(telemetry);
//    }
//
//
//    public static class Paths {
//        public PathChain ToShoot;
//        public PathChain IntakeFirstSet;
//        public PathChain OpenGate;
//        public PathChain ToShoot2;
//        public PathChain IntakeSecondSet;
//        public PathChain ToShoot3;
//        public PathChain IntakeThirdSet;
//        public PathChain ToShoot4;
//        public PathChain LeaveLaunchPad;
//
//        public Paths(Follower follower) {
//            ToShoot = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(33.863, 135.233),
//
//                                    new Pose(53.000, 90.000)
//                            )
//                    ).setConstantHeadingInterpolation(Math.toRadians(180))
//
//                    .build();
//
//            IntakeFirstSet = follower.pathBuilder().addPath(
//                            new BezierCurve(
//                                    new Pose(53.000, 90.000),
//                                    new Pose(45.500, 84.500),
//                                    new Pose(27.000, 84.000)
//                            )
//                    ).setConstantHeadingInterpolation(Math.toRadians(180))
//
//                    .build();
//
//            OpenGate = follower.pathBuilder().addPath(
//                            new BezierCurve(
//                                    new Pose(27.000, 84.000),
//                                    new Pose(24.660, 76.616),
//                                    new Pose(20.737, 72.326)
//                            )
//                    ).setConstantHeadingInterpolation(Math.toRadians(180))
//
//                    .build();
//
//            ToShoot2 = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(20.737, 72.326),
//
//                                    new Pose(53.000, 90.000)
//                            )
//                    ).setConstantHeadingInterpolation(Math.toRadians(180))
//
//                    .build();
//
//            IntakeSecondSet = follower.pathBuilder().addPath(
//                            new BezierCurve(
//                                    new Pose(53.000, 90.000),
//                                    new Pose(52.000, 73.000),
//                                    new Pose(60.000, 59.500),
//                                    new Pose(24.000, 60.000)
//                            )
//                    ).setConstantHeadingInterpolation(Math.toRadians(180))
//
//                    .build();
//
//            ToShoot3 = follower.pathBuilder().addPath(
//                            new BezierCurve(
//                                    new Pose(24.000, 60.000),
//                                    new Pose(33.000, 76.000),
//                                    new Pose(53.000, 90.000)
//                            )
//                    ).setConstantHeadingInterpolation(Math.toRadians(180))
//
//                    .build();
//
//            IntakeThirdSet = follower.pathBuilder().addPath(
//                            new BezierCurve(
//                                    new Pose(53.000, 90.000),
//                                    new Pose(61.500, 30.500),
//                                    new Pose(44.500, 35.500),
//                                    new Pose(24.000, 36.000)
//                            )
//                    ).setConstantHeadingInterpolation(Math.toRadians(180))
//
//                    .build();
//
//            ToShoot4 = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(24.000, 36.000),
//
//                                    new Pose(53.000, 90.000)
//                            )
//                    ).setConstantHeadingInterpolation(Math.toRadians(180))
//
//                    .build();
//
//            LeaveLaunchPad = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(53.000, 90.000),
//
//                                    new Pose(39.000, 77.000)
//                            )
//                    ).setConstantHeadingInterpolation(Math.toRadians(180))
//
//                    .build();
//        }
//    }
//
//
//    public int autonomousPathUpdate() {
//        switch (pathState) {
//
//            case 0: // Start: go to first shoot
//                // Start first path once at beginning
//                follower.followPath(paths.ToShoot);
//                pathState = 1;
//                break;
//
//            case 1: // Wait for ToShoot to finish, then start IntakeFirstSet
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.IntakeFirstSet);
//                    pathState = 2;
//                }
//                break;
//
//            case 2: // Wait for IntakeFirstSet to finish, then go open gate
//                if (!follower.isBusy()) {
//                    follower.setMaxPower(.4);
//                    follower.followPath(paths.O);
//                    pathState = 3;
//                }
//                break;
//
//            case 3: // Wait for OpenGate to finish, then go back to shoot (ToShoot2)
//                if (!follower.isBusy()) {
//                    follower.setMaxPower(.7);
//                    follower.followPath(paths.ToShoot2);
//                    pathState = 4;
//                }
//                break;
//
//            case 4: // Wait for ToShoot2, then run IntakeSecondSet
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.IntakeSecondSet);
//                    pathState = 5;
//                }
//                break;
//
//            case 5: // Wait for IntakeSecondSet, then ToShoot3
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.ToShoot3);
//                    pathState = 6;
//                }
//                break;
//
//            case 6: // Wait for ToShoot3, then IntakeThirdSet
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.IntakeThirdSet);
//                    pathState = 7;
//                }
//                break;
//
//            case 7: // Wait for IntakeThirdSet, then ToShoot4
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.ToShoot4);
//                    pathState = 8;
//                }
//                break;
//
//            case 8: // Wait for ToShoot4, then LeaveLaunchPad
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.LeaveLaunchPad);
//                    pathState = 9;
//                }
//                break;
//
//            case 9: // Wait for LeaveLaunchPad, then stop advancing states
//                if (!follower.isBusy()) {
//                    // End of auto. You can add any final actions here.
//                    // requestOpModeStop(); // Only if you convert to LinearOpMode or use custom stop logic.
//                    pathState = 10; // Idle
//                }
//                break;
//
//            case 10: // Idle, everything done
//            default:
//                // Do nothing, paths complete
//                break;
//        }
//
//        return pathState;
//    }
//
//}
//