package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.helper.Data;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.helper.Alliance;

public class Robot {

    public final Drivetrain drivetrain;
    public final Intake intake;
    public final Outtake outtake;
    public final Turret turret;
    public final Feeder feeder;
    public final Gate gate;
    public final Hood hood;

    public final Alliance alliance;

    public static Pose defaultPose = new Pose(56.5, 8.5, Math.toRadians(90)); // Blue allience park zone
    public static Pose shootTarget = new Pose(3.78, 140.5, 0);

    public Robot(HardwareMap hw, Alliance alliance, boolean drivetrainOn) {
        this.alliance = alliance;

        Pose startPose;
//
        if (alliance == Alliance.RED) {
            shootTarget = new Pose(11, 137.5, 0).mirror();
            defaultPose = defaultPose.mirror();
        }

        if (Data.hasAutoData && Data.getAutoEndPose() != null) {
            startPose = Data.getAutoEndPose();
        } else {
            startPose = defaultPose;
        }

        intake = new Intake(hw);
        outtake = new Outtake(hw);
        turret = new Turret(hw);
        feeder = new Feeder(hw);
        gate = new Gate(hw);
        hood = new Hood(hw);

        drivetrain = null;
    }

    public Robot(HardwareMap hw, Alliance alliance) {
        this.alliance = alliance;

        Pose startPose;
//
        if (alliance == Alliance.RED) {
            shootTarget = new Pose(11, 137.5, 0).mirror();
            defaultPose = defaultPose.mirror();
        }

        if (Data.hasAutoData && Data.getAutoEndPose() != null) {
            startPose = Data.getAutoEndPose();
        } else {
            startPose = defaultPose;
        }


        drivetrain = new Drivetrain(
                hw,
                alliance
        );
        drivetrain.setStart(startPose);
        intake = new Intake(hw);
        outtake = new Outtake(hw);
        turret = new Turret(hw);
        feeder = new Feeder(hw);
        gate = new Gate(hw);
        hood = new Hood(hw);

        drivetrain.startDrive();
    }

    public void autoPeriodic() {
        turret.updateWithVisionAssist(true);
        outtake.periodic();
    }

    public double getDistanceFromTarget() {
        return shootTarget.distanceFrom(drivetrain.getPose());
    }

//    public void adjustSpeedAutomatically(double distInches) {
////        // y = 0.1436x^2 - 5.08065x + 3679.19483
////        double rpm = 9.16492 * distInches + 3581.43175;
////
//////         y = 0.21568 * sin(0.0433515x - 2.54881) + 0.236258
//////        double rightPos =
//////                0.0778295 * Math.sin(.235227 * distInches + .85403)
//////                        + 0.85403;
//////        double leftPos = (0.00403668*distInches)-0.00964995;
////
////        // y = 0.216576 * sin(0.043305x + 0.595078) + 0.762743
////        double leftPos =
////                0.216576 * Math.sin(0.235227 * distInches - 2.94949)
////                        + .14597;
////        double rightPos = 1-leftPos;
////
//////        double rightPos = (-0.00405751*distInches)+1.00987;
//
//        rpm = Math.max(0, rpm);
//        leftPos  = Math.max(0.0, Math.min(1.0, leftPos));
//        rightPos = Math.max(0.0, Math.min(1.0, rightPos));
//
//        outtake.setTargetRPM(rpm);
//        hood.set(leftPos, rightPos);
//    }

public void adjustSpeedAutomatically(double distInches) {

    // LEFT hood servo (quartic)
    // y = 6.83688e-8 x^4 - 0.0000245371 x^3 + 0.00314857 x^2 - 0.16488 x + 3.00982
    double leftPos =
            (6.83688e-8) * Math.pow(distInches, 4)
                    - 0.0000245371 * Math.pow(distInches, 3)
                    + 0.00314857 * distInches * distInches
                    - 0.16488 * distInches
                    + 3.00982;

    // SHOOTER RPM (quartic)
    // y = -0.0000822196 x^4 + 0.0312467 x^3 - 4.40085 x^2 + 290.04881 x - 3290.83485
    double rpm =
            -0.0000822196 * Math.pow(distInches, 4)
                    + 0.0312467  * Math.pow(distInches, 3)
                    - 4.40085    * distInches * distInches
                    + 290.04881  * distInches
                    - 3290.83485;

    double rightPos = 1.0 - leftPos;

    // Safety clamps
    rpm = Math.max(0, rpm);
    leftPos  = Math.max(0.0, Math.min(1.0, leftPos));
    rightPos = Math.max(0.0, Math.min(1.0, rightPos));

    outtake.setTargetRPM(rpm);
    hood.set(leftPos, rightPos);
}






    public void periodic(boolean allowVision) {
        drivetrain.periodic();
        turret.periodic();
        turret.updateWithVisionAssist(allowVision);
        outtake.periodic();
    }

    public void setUpRapidFire() {
        outtake.setTargetRPM(4400);
        hood.set(.1, .9);

    }


    public void drive(com.qualcomm.robotcore.hardware.Gamepad gp) {
        drivetrain.drive(gp);
    }


    public void manualTurret(double power) {
        turret.manual(power);
    }
    public void autoTurret() {
        turret.automatic();
    }

    public void autoAim() {
        turret.face(shootTarget, drivetrain.getPose());
    }

    public void shootHigh() {
        outtake.shootHigh();
    }

    public void shootLow() {
        outtake.shootLow();
//        hood.set(.1, .9);
    }

    public void stopShooter() {
        outtake.stop();
    }
    public boolean shooterReady() {
        return outtake.atTarget();
    }
    public void resetDrivePos() {
        drivetrain.cornerReset();
    }
    public void resetDrivePosAtGoal() {drivetrain.goalReset();}

    public void intakeIn()  { intake.spinIn(); }
    public void intakeOut() { intake.spinOut(); }
    public void intakeOff() { intake.spinOff(); }
    public Pose getPose() {
        return drivetrain.getPose();
    }
    public Pose getShootTarget() {
        return shootTarget;
    }
}
