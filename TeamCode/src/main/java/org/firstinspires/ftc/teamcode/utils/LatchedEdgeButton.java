package org.firstinspires.ftc.teamcode.utils;

public class LatchedEdgeButton {

    private final double holdSeconds;
    private boolean lastState = false;
    private boolean pending = false;
    private double pendingUntil = 0;

    public LatchedEdgeButton(double holdSeconds) {
        this.holdSeconds = holdSeconds;
    }

    public void update(boolean currentState, double nowSeconds) {
        if (currentState && !lastState) {
            pending = true;
            pendingUntil = nowSeconds + holdSeconds;
        } else if (pending && nowSeconds > pendingUntil) {
            pending = false;
        }

        lastState = currentState;
    }

    public boolean consume(double nowSeconds) {
        if (pending && nowSeconds <= pendingUntil) {
            pending = false;
            return true;
        }

        if (pending && nowSeconds > pendingUntil) {
            pending = false;
        }
        return false;
    }
}
