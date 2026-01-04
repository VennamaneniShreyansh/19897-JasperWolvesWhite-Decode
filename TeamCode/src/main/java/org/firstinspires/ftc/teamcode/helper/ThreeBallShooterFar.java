package org.firstinspires.ftc.teamcode.helper;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
public class ThreeBallShooterFar {
    private final Intake intake;
    private final Outtake outtake;
    private static final int SPINUP_MS = 1500;    // 1.5s spinup
    private static final int INTAKE_ON_MS = 124;  // 0.3s feed
    private static final int LAST_INTAKE_ON_MS = 1000;
    private static final int INTAKE_OFF_MS = 600; // 0.5s pause
    private static final int MAX_BALLS = 3;

    public boolean shootingActive = false;
    public boolean shootingDone = false;
    public int ballsShot = 0;
    public int stage = 0;
    public long stageStartTime = 0;

    public ThreeBallShooterFar(Intake intake, Outtake outtake) {
        this.intake = intake;
        this.outtake = outtake;
    }

    public void start() {
        shootingActive = true;
        shootingDone = false;
        ballsShot = 0;
        stage = 0;
        stageStartTime = System.currentTimeMillis();
        outtake.shootHigh(); // Start flywheel
    }

    public boolean isActive() { return shootingActive && !shootingDone; }
    public boolean isDone() { return shootingDone; }

    public void update() {
        if (!shootingActive || shootingDone) return;
        long now = System.currentTimeMillis();

        switch (stage) {

            case 0: // SPINUP
                if (now - stageStartTime >= SPINUP_MS) {
                    stage = 1;
                    stageStartTime = now;
                }
                break;

            case 1: // FEED BALL
                intake.spinIn();

                long feedTime = (ballsShot == MAX_BALLS - 1)
                        ? LAST_INTAKE_ON_MS
                        : INTAKE_ON_MS;

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
                        outtake.shootHigh();
                    } else {
                        stage = 1;
                        stageStartTime = now;
                    }
                }
                break;
        }
    }

}