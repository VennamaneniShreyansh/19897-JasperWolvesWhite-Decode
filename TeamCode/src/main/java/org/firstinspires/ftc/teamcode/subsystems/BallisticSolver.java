package org.firstinspires.ftc.teamcode.subsystems;

public class BallisticSolver {

    public static ShotSolution solve(double distance, double height) {

        double g = ShooterConstants.GRAVITY;

        double r = ShooterConstants.PASS_RADIUS;
        double h = ShooterConstants.PASS_HEIGHT;
        double theta = Math.toRadians(ShooterConstants.PASS_ANGLE);

        ShotSolution sol = new ShotSolution();

        // hood angle from pass-through geometry
        double slope = (h - height) / r;

        double hood = Math.atan(slope) - theta;

        hood = Math.max(
                Math.toRadians(ShooterConstants.MIN_HOOD),
                Math.min(Math.toRadians(ShooterConstants.MAX_HOOD), hood)
        );

        sol.hoodAngle = hood;

        // flywheel speed
        double cos = Math.cos(hood);

        double velocity = Math.sqrt(
                (g * distance * distance) /
                        (2 * cos * cos * (distance * Math.tan(hood) - height))
        );

        sol.flywheelVelocity = velocity;

        // airtime
        sol.airTime = distance / (velocity * cos);

        return sol;
    }
}