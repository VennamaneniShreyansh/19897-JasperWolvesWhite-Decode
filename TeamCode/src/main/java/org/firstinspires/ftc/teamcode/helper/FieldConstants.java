package org.firstinspires.ftc.teamcode.helper;

import com.pedropathing.geometry.Pose;

public final class FieldConstants {
    private static final Pose BLUE_START = new Pose(56.5, 8.5, Math.toRadians(90));
    private static final Pose BLUE_GOAL  = new Pose(3.78, 140.5, 0);
    private static final Pose BLUE_GOAL_FAR = new Pose(10, 143, 0);
    private static final Pose BLUE_GOAL_RIGHT_FAR = new Pose(15, 1453, 0);
    public static Pose startPose(Alliance alliance) {
        return mirrorIfRed(BLUE_START, alliance);
    }

    public static Pose goalPose(Alliance alliance) {
        return mirrorIfRed(BLUE_GOAL, alliance);
    }
    public static Pose farGoalPose(Alliance alliance) {
        return mirrorIfRed(BLUE_GOAL_FAR, alliance);
    }
    public static Pose moreRightFarGoalPose(Alliance alliance) {
        return mirrorIfRed(BLUE_GOAL_RIGHT_FAR, alliance);
    }

    private static Pose mirrorIfRed(Pose pose, Alliance alliance) {
        if (alliance.isBlue()) return pose;
        return pose.mirror();
    }

    private FieldConstants() {}
}
