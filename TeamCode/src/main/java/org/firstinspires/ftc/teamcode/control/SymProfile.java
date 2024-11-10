package org.firstinspires.ftc.teamcode.control;
public class SymProfile extends AsymProfile {
    public static class SymConstraints extends AsymConstraints {
        public SymConstraints(double vm, double am) {
            super(vm, am, am);
        }
    }
    public SymProfile(SymConstraints constraints, double ti, MotionState i, MotionState f) {
        super(constraints, ti, i, f);
    }
    public static SymProfile extendSym(MotionProfile p, SymConstraints constraints, double ti, MotionState f) {
        return new SymProfile(constraints, ti, p.state(ti), f);
    }
    public static SymProfile extendSym(MotionProfile p, SymConstraints constraints, MotionState f) {
        return extendSym(p, constraints, p.tf, f);
    }
}