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
    private final LynxModule hub;

    public static Pose defaultPose = new Pose(8.5, 8.5, Math.toRadians(90)).mirror(); // Blue allience park zone
    public static Pose shootTarget = new Pose(11, 137.5, 0);

    public Robot(HardwareMap hw, Alliance alliance) {
        this.alliance = alliance;

        Pose startPose;

        if (Data.hasAutoData && Data.getAutoEndPose() != null) {
            startPose = Data.getAutoEndPose();
        } else {
            startPose = defaultPose;
        }

        drivetrain = new Drivetrain(
                hw,
                alliance,
                startPose
        );

        intake = new Intake(hw);
        outtake = new Outtake(hw);
        turret = new Turret(hw);
        feeder = new Feeder(hw);
        gate = new Gate(hw);
        hood = new Hood(hw);

        hub = hw.getAll(LynxModule.class).get(0);
        hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        drivetrain.startDrive();

        // Restore turret yaw if we have it
//        if (Data.hasAutoData) {
//            turret.setYaw(Data.turretYaw);
//        }
    }


    public void periodic() {
        hub.clearBulkCache();
        drivetrain.periodic();
//        turret.periodic();
        turret.updateWithVisionAssist(outtake.isEnabled());
        outtake.periodic();
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
    }

    public void stopShooter() {
        outtake.stop();
    }

    public boolean shooterReady() {
        return outtake.atTarget();
    }


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
