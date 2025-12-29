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
    }

    public void autoPeriodic() {
        hub.clearBulkCache();
        turret.updateWithVisionAssist(true);
        outtake.periodic();
    }

    public double getDistanceFromTarget() {
        return shootTarget.distanceFrom(drivetrain.getPose());
    }

    public void adjustSpeedAutomatically(double distInches) {
        // y = 0.1436x^2 - 5.08065x + 3679.19483
        double rpm =
                0.1436 * distInches * distInches
                        - 5.08065 * distInches
                        + 3679.19483;

//         y = 0.21568 * sin(0.0433515x - 2.54881) + 0.236258
        double leftPos =
                0.21568 * Math.sin(0.0433515 * distInches - 2.54881)
                        + 0.236258;
//        double leftPos = (0.00403668*distInches)-0.00964995;

        // y = 0.216576 * sin(0.043305x + 0.595078) + 0.762743
        double rightPos =
                0.216576 * Math.sin(0.043305 * distInches + 0.595078)
                        + 0.762743;

//        double rightPos = (-0.00405751*distInches)+1.00987;

        rpm = Math.max(0, rpm);
        leftPos  = Math.max(0.0, Math.min(1.0, leftPos));
        rightPos = Math.max(0.0, Math.min(1.0, rightPos));

        outtake.setTargetRPM(rpm);
        hood.set(leftPos, rightPos);
    }



    public void periodic(boolean allowVision) {
        hub.clearBulkCache();
        drivetrain.periodic();
        turret.periodic();
        turret.updateWithVisionAssist(allowVision);
//        outtake.periodic();
    }

    public void setUpRapidFire() {
        outtake.setTargetRPM(4250);
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
