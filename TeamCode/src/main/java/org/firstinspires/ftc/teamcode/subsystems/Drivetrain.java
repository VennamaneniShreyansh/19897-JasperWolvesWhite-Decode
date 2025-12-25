package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.helper.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Drivetrain {
    private Follower follower;
    private final Alliance alliance;
    public boolean hold = false, fieldCentric = true;
    public Drivetrain(HardwareMap hardwareMap, Alliance a, Pose start) {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(start);
        this.alliance = a;
    }

    public void startDrive() {
        follower.startTeleopDrive();
    }

    public void periodic() {
        follower.update();
    }

    public void drive(Gamepad gamepad) {
        if (!hold)
            if (fieldCentric)
                follower.setTeleOpDrive(-gamepad.left_stick_y, -gamepad.left_stick_x, -gamepad.right_stick_x, false, alliance == Alliance.BLUE ? Math.toRadians(180) : 0);
            else
                follower.setTeleOpDrive(-gamepad.left_stick_y, -gamepad.left_stick_x, -gamepad.right_stick_x, true);
    }

    public void holdCurrent() {
        follower.holdPoint(new BezierPoint(follower.getPose()), follower.getHeading(), true);
        hold = true;
    }

    public void releaseHold() {
        hold = false;
    }

    public void teleToggleCentric() {
        fieldCentric = !fieldCentric;
    }

    public void cornerReset() {
        if (alliance.equals(Alliance.BLUE))
            follower.setPose(new Pose(8.5, 8.5, Math.toRadians(90)).mirror()); // Blue park
        else
            follower.setPose(new Pose(8.5, 8.5, Math.toRadians(90)));
    }


    public void setStart(Pose start) {
        follower.setStartingPose(start);
    }

    public Pose getPose() {
        return follower.getPose();
    }


    public double getT() {
        return follower.getCurrentTValue();
    }

    public Follower getFollower() { return follower;}
}