package org.firstinspires.ftc.teamcode.movement;
import org.firstinspires.ftc.teamcode.control.MotionState;
public class CubicSpline {
    public final MotionState i;
    public final MotionState f;
    public CubicSpline(MotionState i, MotionState f) {
        this.i = i;
        this.f = f;
    }
    public MotionState state(double t) {
        return new MotionState(i.x * (2*t*t*t - 3*t*t + 1) + i.v * (t*t*t - 2*t*t + t) + f.x * (-2*t*t*t + 3*t*t) + f.x * (t*t*t - t*t),
                i.x * (6*t*t - 6*t) + i.v * (3*t*t - 4*t + 1) + f.x * (-6*t*t + 6*t) + f.v * (3*t*t - 2*t),
                i.x * (12*t - 6) + i.v * (6*t - 4) + f.x * (-12*t + 6) + f.v * (6*t - 2));
    }
}