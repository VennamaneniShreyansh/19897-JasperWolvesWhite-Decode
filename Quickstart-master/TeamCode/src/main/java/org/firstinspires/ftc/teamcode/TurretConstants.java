package org.firstinspires.ftc.teamcode;

public class TurretConstants {

    public static final double TICKS_PER_MOTOR_REV = 383.6;
    public static final double GEAR_RATIO = 2.5; // 10:4

    public static final double TICKS_PER_TURRET_REV =
            TICKS_PER_MOTOR_REV * GEAR_RATIO;

    public static final double TICKS_PER_DEGREE =
            TICKS_PER_TURRET_REV / 360.0;
}
