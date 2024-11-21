package org.firstinspires.ftc.teamcode.control;
public class RampProfile extends MotionProfile {
    private MotionState f;
    public RampProfile(double ti, MotionState i, MotionState f, double dt) {
        this.ti = ti;
        this.i = i;
        this.tf = ti + dt;
        this.f = f;
    }
    public MotionState state(double t) {
        if (t < ti) {
            return new MotionState(i.x);
        } else if (t < tf) {
            return new MotionState(i.x + (f.x - i.x) * (t - ti) / (tf - ti), (f.x - i.x) / (tf - ti));
        }
        return new MotionState(f.x);
    }
    public static RampProfile extendRamp(MotionProfile p, double ti, MotionState f, double dt) {
        return new RampProfile(ti, p.state(ti), f, dt);
    }
    public static RampProfile extendRamp(MotionProfile p, MotionState f, double dt) {
        return extendRamp(p, p.ti, f, dt);
    }
}
