package org.firstinspires.ftc.teamcode.helper;

public enum Alliance {
    BLUE(1),
    RED(-1);

    private final int ySign;

    Alliance(int ySign) {
        this.ySign = ySign;
    }

    public int ySign() {
        return ySign;
    }

    public boolean isBlue() { return this == BLUE; }
    public boolean isRed()  { return this == RED;  }
}
