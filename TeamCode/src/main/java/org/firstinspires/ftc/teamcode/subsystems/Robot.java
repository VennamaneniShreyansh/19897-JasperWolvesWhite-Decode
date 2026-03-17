package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.helper.Data;
import org.firstinspires.ftc.teamcode.helper.Alliance;
import org.firstinspires.ftc.teamcode.helper.FieldConstants;
//import org.firstinspires.ftc.teamcode.references.Feeder;
import org.firstinspires.ftc.teamcode.subsystems.Gate;

import java.lang.reflect.Field;

public class Robot {

    public final Drivetrain drivetrain;
    public final Intake intake;
    public final Outtake outtake;
    public final Turret turret;
//    public final Feeder feeder;
    public final Gate gate;
    public final Hood hood;
    public final GobuildaRGB rgb;


    private final Alliance alliance;
    public Pose shootTarget = null;


    public Robot(HardwareMap hw, Alliance alliance, boolean drivetrainOn) {
        this.alliance = alliance;

        shootTarget = FieldConstants.farGoalPose(alliance);

        intake = new Intake(hw);
        outtake = new Outtake(hw);
        turret = new Turret(hw);
//        feeder = new Feeder(hw);
        gate = new Gate(hw);
        hood = new Hood(hw);
        rgb = new GobuildaRGB(hw);

        drivetrain = null;
    }



    public Robot(HardwareMap hw, Alliance alliance) {
        this.alliance = alliance;

        Pose startPose = Data.hasAutoData && Data.getAutoEndPose() != null
                ? Data.getAutoEndPose()
                : FieldConstants.startPose(alliance);

        shootTarget = FieldConstants.goalPose(alliance);

        drivetrain = new Drivetrain(hw, alliance);
        drivetrain.setStart(startPose);

        intake = new Intake(hw);
        outtake = new Outtake(hw);
        turret = new Turret(hw);
//        feeder = new Feeder(hw);
        gate = new Gate(hw);
        hood = new Hood(hw);
        rgb = new GobuildaRGB(hw);

        drivetrain.startDrive();
    }

    public void updateRPMIndicator() {
        if (!outtake.isEnabled() || outtake.getTarget() == 0) {
            rgb.setOff();  // or team color, etc.
            return;
        }

        double target = outtake.getTarget();
        double actual = (outtake.getRPMLeft() + outtake.getRPMRight()) / 2.0;
        double error = Math.abs(target - actual);

        if (error > 400) {
            rgb.setRed();
        } else if (error > 200) {
            rgb.setYellow();
        } else if (error <= 100) {
            rgb.setGreen();
        }
    }
    public double getDistanceFromTarget() {
        return shootTarget.distanceFrom(drivetrain.getPose());
    }
    public void adjustSpeedAutomatically(double distInches) {



        if (drivetrain.isOnBottomHalf()) {
            double rpm = (13.98917*distInches)+2143.59206;
            outtake.setTargetRPM(rpm);
            hood.set(.05);
        } else {
            // RIGHT hood servo (cubic)
            // y = -0.0000073169 x^3 + 0.00195206 x^2 - 0.17015 x + 5.03082
            double rightPos =
                    -0.0000073169 * Math.pow(distInches, 3)
                            + 0.00195206 * distInches * distInches
                            - 0.17015    * distInches
                            + 5.03082;

            // SHOOTER RPM (cubic)
            // y = 0.00238598 x^3 - 0.644182 x^2 + 69.78364 x + 951.13353
            double rpm =
                    0.00238598 * Math.pow(distInches, 3)
                            - 0.644182  * distInches * distInches
                            + 69.78364  * distInches
                            + 951.13353;

//        double leftPos = 1.0 - rightPos;

            // Safety clamps
            rpm = Math.max(0, rpm);
//        leftPos  = Math.max(0.0, Math.min(1.0, leftPos));
            rightPos = Math.max(0.0, Math.min(1.0, rightPos));
            outtake.setTargetRPM(rpm);
            hood.set(rightPos);
        }


    }


    public void periodic() {
        if (drivetrain != null) drivetrain.periodic();
        turret.periodic();
        outtake.periodic();
        trackDrivetrain();
        updateRPMIndicator();
    }

    public void autoAimWithFollower(Pose robotPose) {
        if (shootTarget != null) {
//            turret.face(shootTarget, robotPose, drivetrain.getCurrentVelocity());
            turret.face(shootTarget, robotPose);
        }
    }

    public void trackDrivetrain() {
        // Skip tracking if no drivetrain (autonomous case)
        if (drivetrain == null) {
            if (shootTarget == null) {
                shootTarget = FieldConstants.farGoalPose(alliance);
            }
            return;
        }

        // Normal teleop tracking logic
        if (drivetrain.isOnBottomHalf() && shootTarget != FieldConstants.farGoalPose(alliance)) {
            shootTarget = FieldConstants.farGoalPose(alliance);
        } else if (!drivetrain.isOnBottomHalf() && shootTarget != FieldConstants.goalPose(alliance)) {
            shootTarget = FieldConstants.goalPose(alliance);
        }
    }


    public void drive(com.qualcomm.robotcore.hardware.Gamepad gp) {
        drivetrain.drive(gp);
    }
    public void autoTurret() {
        turret.automatic();
    }
    public void autoAim() {
//        turret.face(shootTarget, drivetrain.getPose(), drivetrain.getCurrentVelocity()); // SOTM
        turret.face(shootTarget, drivetrain.getPose());
    }
    public void stopTurretAim() {turret.setTurretTarget(0);}

    public void shootHigh() {
        outtake.shootHigh();
    }
    public void shootLow() { outtake.shootLow(); }
    public void stopShooter() {
        outtake.stop();
    }
    public void resetDrivePos() {
        drivetrain.cornerReset();
    }
    public void resetDrivePosAtGoal() {drivetrain.goalReset();}
    public void resetDrivePosBackZone() {drivetrain.resetDrivePosBackZone();}

    public void intakeIn()  { intake.spinIn(); }
    public void intakeOut() { intake.spinOut(); }
    public void intakeOff() { intake.spinOff(); }
    public void slowIntakeIn() { intake.slowSpinIn(); }
    public Pose getPose() {
        return drivetrain.getPose();
    }
    public Pose getShootTarget() {
        return shootTarget;
    }
}
