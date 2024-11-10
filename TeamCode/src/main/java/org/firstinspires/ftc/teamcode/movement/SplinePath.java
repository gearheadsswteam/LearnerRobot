package org.firstinspires.ftc.teamcode.movement;
import static java.lang.Math.*;
import org.firstinspires.ftc.teamcode.control.MotionState;

public class SplinePath implements Path {
    public static final int APPROX_PTS = 5;
    public static final double EPS = 1e-6;
    private double len;
    private CubicSpline x;
    private CubicSpline y;
    private CubicSplineInterpolator arcLen;
    public SplinePath(Vec xi, Vec vi, Vec xf, Vec vf) {
        x = new CubicSpline(new MotionState(xi.x, vi.x), new MotionState(xf.x, vf.x));
        y = new CubicSpline(new MotionState(xi.y, vi.y), new MotionState(xf.y, vf.y));
        double[] tArr = new double[APPROX_PTS + 1];
        MotionState[] sArr = new MotionState[APPROX_PTS + 1];
        AdaptiveQuadrature integrator = new AdaptiveQuadrature(t -> {
            MotionState sx = x.state(t);
            MotionState sy = y.state(t);
            return sqrt(sx.v*sx.v + sy.v*sy.v);
        });
        for (int i = 0; i <= APPROX_PTS; i++) {
            double t = (double)i / APPROX_PTS;
            tArr[i] = integrator.integrate(0, t, EPS);
            MotionState sx = x.state(t);
            MotionState sy = y.state(t);
            sArr[i] = new MotionState(t, 1 / sqrt(sx.v*sx.v + sy.v*sy.v));
        }
        len = tArr[APPROX_PTS];
        arcLen = new CubicSplineInterpolator(tArr, sArr);
    }
    @Override public PathState state(double t) {
        double ta = arcLen.get(t * len);
        MotionState sx = x.state(ta);
        MotionState sy = y.state(ta);
        return new PathState(new Vec(sx.x, sy.x), new Vec(sx.v, sy.v).normalize(),
                (sx.v * sy.a - sx.a * sy.v) / pow(sx.v*sx.v + sy.v*sy.v, 1.5));
    }
    @Override
    public double length() {
        return len;
    }
}