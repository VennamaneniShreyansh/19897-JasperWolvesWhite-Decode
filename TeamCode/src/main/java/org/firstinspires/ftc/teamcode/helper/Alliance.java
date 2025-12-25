package org.firstinspires.ftc.teamcode.helper;

public enum Alliance {
    RED,
    BLUE;

    public boolean isBlue() {
        return this == BLUE;
    }

    public boolean isRed() {
        return this == RED;
    }
}
