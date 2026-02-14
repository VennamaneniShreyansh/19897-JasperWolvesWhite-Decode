package org.firstinspires.ftc.teamcode.helper;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

public class ThreeBallShooterFar {
    private final Intake intake;
    private final Outtake outtake;
    private static final int SPINUP_MS = 500;    // 1.5s spinup
    private static final int LONG_SPINUP_MS = 3000;
    private static final int INTAKE_ON_MS = 100;  // 0.3s feed
    private static final int SECOND_INTAKE_ON_MS = 120;
    private static final int LAST_INTAKE_ON_MS = 1000;
    private static final int INTAKE_OFF_MS = 600; // 0.5s pause
    private static final int MAX_BALLS = 3;

    public boolean shootingActive = false;
    public boolean shootingDone = false;
    public int ballsShot = 0;
    public int stage = 0;
    public long stageStartTime = 0;
    public boolean cycleBool = false;

    public ThreeBallShooterFar(Intake intake, Outtake outtake) {
        this.intake = intake;
        this.outtake = outtake;
    }

    public void start(boolean condition) {
        shootingActive = true;
        shootingDone = false;
        ballsShot = 0;
        stage = 0;
        stageStartTime = System.currentTimeMillis();
        outtake.shootDoubleHigh(); // Start flywheel
        cycleBool = condition;
    }

    public boolean isActive() { return shootingActive && !shootingDone; }
    public boolean isDone() { return shootingDone; }

    public void update() {
        if (!shootingActive || shootingDone) return;
        long now = System.currentTimeMillis();

        switch (stage) {



            case 0: // SPINUP
                int spinup = (ballsShot == 0 && cycleBool) ? LONG_SPINUP_MS : SPINUP_MS;
                if (now - stageStartTime >= spinup) {
                    stage = 1;
                    stageStartTime = now;
                }
                break;

            case 1: // FEED BALL
                intake.spinIn();

                long feedTime = INTAKE_ON_MS;
                if (ballsShot == MAX_BALLS - 1) {
                    feedTime = LAST_INTAKE_ON_MS;
                } else if (ballsShot == MAX_BALLS - 2) {
                    feedTime = SECOND_INTAKE_ON_MS;
                }

                if (now - stageStartTime >= feedTime) {
                    stage = 2;
                    stageStartTime = now;
                }
                break;

            case 2: // PAUSE
                intake.spinOff();
                if (now - stageStartTime >= INTAKE_OFF_MS) {
                    ballsShot++;
                    if (ballsShot >= MAX_BALLS) {
                        shootingActive = false;
                        shootingDone = true;
                        intake.spinOff();
                    } else {
                        outtake.shootHigh();
                        stage = 1;
                        stageStartTime = now;
                    }
                }
                break;
        }
    }

//    public void update() {
//        if (!shootingActive || shootingDone) return;
//        long now = System.currentTimeMillis();
//
//        // FORCE MAX POWER EVERY LOOP during spinup - overrides everything
//        if (stage == 0) {
//            outtake.shootDoubleHigh();  // Hammer it
//            Outtake.kV = 94;
//            double leftRPM = outtake.getRPMLeft();
//            double rightRPM = outtake.getRPMRight();
//
//            // 95% of 5500 = 5225 RPM threshold
//            if (leftRPM >= 4800 && rightRPM >= 4800) {
//                stage = 1;
//                stageStartTime = now;
//            }
//            return;  // Exit early - max aggression
//        }
//
//        switch (stage) {
//            case 1: // FEED BALL
//                intake.spinIn();
//                long feedTime = (ballsShot == MAX_BALLS - 1) ? LAST_INTAKE_ON_MS : INTAKE_ON_MS;
//                if (now - stageStartTime >= feedTime) {
//                    stage = 2;
//                    stageStartTime = now;
//                }
//                break;
//
//            case 2: // PAUSE
//                intake.spinOff();
//                if (now - stageStartTime >= INTAKE_OFF_MS) {
//                    ballsShot++;
//                    if (ballsShot >= MAX_BALLS) {
//                        shootingActive = false;
//                        shootingDone = true;
//                        intake.spinOff();
//                    } else {
//                        stage = 1;
//                        stageStartTime = now;
//                    }
//                }
//                break;
//        }
//    }


}