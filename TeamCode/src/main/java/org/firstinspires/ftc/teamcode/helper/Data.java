package org.firstinspires.ftc.teamcode.helper;

import com.pedropathing.geometry.Pose;

public class Data {
    public static Pose autoEndPose = null;
    public static double turretYaw = 0.0;
    public static boolean hasAutoData = false;
    public static void clear() {
        autoEndPose = null;
        turretYaw = 0.0;
        hasAutoData = false;
    }

    public static Pose getAutoEndPose() {
        return autoEndPose;
    }
}
