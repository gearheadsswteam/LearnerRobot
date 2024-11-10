package org.firstinspires.ftc.teamcode.movement;

import org.firstinspires.ftc.teamcode.control.MotionState;

public class CubicSplineInterpolator {
    private double[] t;
    private MotionState i;
    private MotionState f;
    private CubicSpline[] splines;
    public CubicSplineInterpolator(double[] t, MotionState[] states) {
        this.t = t;
        i = states[0];
        f = states[t.length - 1];
        splines = new CubicSpline[t.length - 1];
        for (int i = 0; i < splines.length; i++) {
            splines[i] = new CubicSpline(new MotionState(states[i].x, states[i].v * (t[i + 1] - t[i])),
                    new MotionState(states[i + 1].x, states[i + 1].v * (t[i + 1] - t[i])));
        }
    }
    public double get(double time) {
        if (time < t[0]) {
            return i.x + i.v * (time - t[0]);
        }
        for (int i = 1; i < t.length; i++) {
            if (time < t[i]) {
                return splines[i - 1].state((time - t[i - 1]) / (t[i] - t[i - 1])).x;
            }
        }
        return f.x + f.v * (time - t[t.length - 1]);
    }
}