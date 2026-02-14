package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.helper.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Drivetrain {
    private Follower follower;
    private final Alliance alliance;
    public boolean hold = false, fieldCentric = false;
    public Drivetrain(HardwareMap hardwareMap, Alliance a) {
        follower = Constants.createFollower(hardwareMap);

        this.alliance = a;
    }

    public void startDrive() {
        follower.startTeleopDrive(true);
    }

    public void periodic() {
        follower.update();
    }

    public void drive(Gamepad gamepad) {
        if (!hold)
            if (fieldCentric)
                follower.setTeleOpDrive(-gamepad.left_stick_y, -gamepad.left_stick_x, -gamepad.right_stick_x, false, alliance == Alliance.BLUE ? Math.toRadians(180) : 0);
            else
                follower.setTeleOpDrive(-gamepad.left_stick_y, -gamepad.left_stick_x, -gamepad.right_stick_x*.7, true);
    }

    public void holdCurrent() {
        follower.holdPoint(new BezierPoint(follower.getPose()), follower.getHeading(), true);
        hold = true;
    }

    public void cornerReset() {
        if (alliance.isBlue())
            follower.setPose(new Pose(8.5, 8.5, Math.toRadians(90)).mirror());
        else
            follower.setPose(new Pose(8.5, 8.5, Math.toRadians(90)));
    }
    public void resetDrivePosBackZone() {
        if (alliance.isBlue())
            follower.setPose(new Pose(64, 8.5, Math.toRadians(180)));
        else
            follower.setPose(new Pose(64, 8.5, Math.toRadians(180)).mirror());
    }
    public void goalReset() {
        if (alliance.isBlue()) {
            follower.setPose(new Pose(32.5, 135, Math.toRadians(180)));
        } else follower.setPose(new Pose(32.5, 135, Math.toRadians(180)).mirror());
    }

    public Vector getCurrentVelocity() {
        return follower.getVelocity();
    }

    public boolean isOnBottomHalf() {
        return follower.getPose().getY() < 40;
    }
    public boolean isInScoreZone() {
        double yPos = follower.getPose().getY();
        double xPos = follower.getPose().getX();
        return (yPos > 60) || (yPos < 40 && xPos > 35 && xPos < 110);
    }

    public void setStart(Pose start) {
        follower.setStartingPose(start);
    }
    public Pose getPose() {
        return follower.getPose();
    }
    public Follower getFollower() { return follower;}
}