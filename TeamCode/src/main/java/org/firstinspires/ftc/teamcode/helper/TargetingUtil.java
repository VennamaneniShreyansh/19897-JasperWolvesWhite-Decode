package org.firstinspires.ftc.teamcode.helper;

import com.pedropathing.geometry.Pose;

public final class TargetingUtil {

    public static Pose compensatedGoal(
            Pose goal,
            Pose velocity,
            double k
    ) {
        return new Pose(
                goal.getX() - velocity.getX() * k,
                goal.getY() - velocity.getY() * k,
                goal.getHeading()
        );
    }
}