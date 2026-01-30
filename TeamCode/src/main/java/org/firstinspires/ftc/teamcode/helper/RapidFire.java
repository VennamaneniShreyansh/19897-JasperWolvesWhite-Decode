package org.firstinspires.ftc.teamcode.helper;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
public class RapidFire {
    private final Intake intake;
    private final Outtake outtake;
    private static final int SPINUP_MS = 500;    // 0.5s spinup
    private static final int INTAKE_ON_MS = 1500;  // 0.3s feed

    public boolean shootingActive = false;
    public boolean shootingDone = false;
    public int ballsShot = 0;
    public int stage = 0;
    public long stageStartTime = 0;

    public RapidFire(Intake intake, Outtake outtake) {
        this.intake = intake;
        this.outtake = outtake;
    }

    public void start() {
        shootingActive = true;
        shootingDone = false;
        ballsShot = 0;
        stage = 0;
        stageStartTime = System.currentTimeMillis();
    }

    public boolean isActive() { return shootingActive && !shootingDone; }
    public boolean isDone() { return shootingDone; }

    public void update() {
        if (!shootingActive || shootingDone) return;
        long now = System.currentTimeMillis();

        switch (stage) {

            case 0:
                if (now - stageStartTime >= SPINUP_MS) {
                    stage = 1;
                    stageStartTime = now;
                }
                break;

            case 1: // FEED BALL
                intake.spinIn();
                if (now - stageStartTime >= INTAKE_ON_MS) {
                    stage = 2;
                    stageStartTime = now;
                }
                break;

            case 2:
                intake.spinOff();
                shootingActive = false;
                shootingDone = true;
                intake.spinOff();


        }
    }

}