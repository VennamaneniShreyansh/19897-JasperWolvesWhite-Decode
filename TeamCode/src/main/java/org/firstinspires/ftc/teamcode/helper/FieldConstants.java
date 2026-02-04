package org.firstinspires.ftc.teamcode.helper;

import com.pedropathing.geometry.Pose;

public final class FieldConstants {
    private static final Pose BLUE_START = new Pose(56.5, 8.5, Math.toRadians(90));
    private static final Pose BLUE_GOAL  = new Pose(3.78, 140.5, 0);

    public static Pose startPose(Alliance alliance) {
        return mirrorIfRed(BLUE_START, alliance);
    }

    public static Pose goalPose(Alliance alliance) {
        return mirrorIfRed(BLUE_GOAL, alliance);
    }

    private static Pose mirrorIfRed(Pose pose, Alliance alliance) {
        if (alliance.isBlue()) return pose;
        return new Pose(
                pose.getX(),
                -pose.getY(),
                Math.PI - pose.getHeading()
        );
    }

    private FieldConstants() {}
}
