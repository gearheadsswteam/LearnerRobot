package org.firstinspires.ftc.teamcode.control;
public class MotionState {
    public final double x;
    public final double v;
    public final double a;
    public MotionState(double x, double v, double a) {
        this.x = x;
        this.v = v;
        this.a = a;
    }
    public MotionState(double x, double v) {
        this(x, v, 0);
    }
}
