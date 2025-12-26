package org.firstinspires.ftc.teamcode.helper;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

public class ThreeBallShooter {

    private final Intake intake;
    private final Outtake outtake;

    private static final long INTAKE_ON_MS  = 300;  // 0.3 seconds
    private static final long INTAKE_OFF_MS = 500;  // 0.5 seconds
    private static final int  MAX_BALLS     = 3;

    private boolean shootingActive = false;
    private boolean shootingDone   = false;
    private int ballsShot = 0;
    private int stage = 0;
    private long stageStartTime = 0;

    public ThreeBallShooter(Intake intake, Outtake outtake) {
        this.intake = intake;
        this.outtake = outtake;
    }

    public void start() {
        shootingActive = true;
        shootingDone   = false;
        ballsShot      = 0;
        stage          = 0;
        stageStartTime = System.currentTimeMillis();

        // Spin up the flywheel to low goal (use shootHigh() if you need higher power)
        outtake.shootLow();
    }

    public boolean isActive() {
        return shootingActive && !shootingDone;
    }

    public boolean isDone() {
        return shootingDone;
    }

    public void update() {
        if (!shootingActive || shootingDone) return;

        long now = System.currentTimeMillis();

        switch (stage) {
            case 0:
                // Wait for outtake RPM to reach target speed
                if (outtake.atTarget()) {
                    stage = 1;
                    stageStartTime = now;
                }
                break;

            case 1:
                // Intake ON for 0.3 seconds
                intake.spinIn();
                if (now - stageStartTime >= INTAKE_ON_MS) {
                    stage = 2;
                    stageStartTime = now;
                }
                break;

            case 2:
                // Intake OFF for 0.5 seconds
                intake.spinOff();
                if (now - stageStartTime >= INTAKE_OFF_MS) {
                    ballsShot++;
                    if (ballsShot >= MAX_BALLS) {
                        // Finished all three balls
                        shootingActive = false;
                        shootingDone   = true;
                        intake.spinOff();
                        outtake.stop();
                    } else {
                        // Start next ball cycle
                        stage = 1;
                        stageStartTime = now;
                    }
                }
                break;
        }
    }
}
