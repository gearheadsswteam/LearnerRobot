package org.firstinspires.ftc.teamcode.movement;
import static java.lang.Math.*;
import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.control.AsyncExtendedKalmanFilter;
public class ThreeWheelKalmanLocalizer extends AsyncExtendedKalmanFilter implements Localizer {
    private final double EPS = 1e-6;
    private Pose vel = new Pose(0, 0, 0);
    private Encoder enc1;
    private Encoder enc2;
    private Encoder enc3;
    private double last1;
    private double last2;
    private double last3;
    private double f;
    private SimpleMatrix kin;
    private SimpleMatrix exp = new SimpleMatrix(3, 3);
    private SimpleMatrix rot = new SimpleMatrix(3, 3);
    private SimpleMatrix Q;
    public ThreeWheelKalmanLocalizer(Encoder enc1, Encoder enc2, Encoder enc3, Pose p1, Pose p2, Pose p3, double f, double window, SimpleMatrix Q) {
        super(window);
        this.enc1 = enc1;
        this.enc2 = enc2;
        this.enc3 = enc3;
        this.f = f;
        kin = new SimpleMatrix(new double[][] {
                {cos(p1.h), sin(p1.h), p1.x * sin(p1.h) - p1.y * cos(p1.h)},
                {cos(p2.h), sin(p2.h), p2.x * sin(p2.h) - p2.y * cos(p2.h)},
                {cos(p3.h), sin(p3.h), p3.x * sin(p3.h) - p3.y * cos(p3.h)}}).invert();
        exp.set(2, 2, 1);
        rot.set(2, 2, 1);
        this.Q = Q;
    }
    @Override
    public SimpleMatrix measure(double time) {
        double p1 = f * enc1.getPosition();
        double p2 = f * enc2.getPosition();
        double p3 = f * enc3.getPosition();
        if (!states.isEmpty()) {
            SimpleMatrix local = kin.mult(new SimpleMatrix(new double[]{p1 - last1, p2 - last2, p3 - last3}));
            rot.set(0, 0, cos(state().get(2)));
            rot.set(0, 1, -sin(state().get(2)));
            rot.set(1, 0, sin(state().get(2)));
            rot.set(1, 1, cos(state().get(2)));
            vel = new Pose(rot.mult(local).scale(1 / (time - states.getLast().time)));
            last1 = p1;
            last2 = p2;
            last3 = p3;
            return local;
        }
        return new SimpleMatrix(3, 1);
    }
    @Override
    public SimpleMatrix f(SimpleMatrix x, SimpleMatrix u) {
        if (abs(u.get(2)) < EPS) {
            exp.set(0, 0, 1 - u.get(2) * u.get(2) / 6);
            exp.set(0, 1, -u.get(2) / 2);
            exp.set(1, 0, u.get(2) / 2);
            exp.set(1, 1, 1 - u.get(2) * u.get(2) / 6);
        } else {
            exp.set(0, 0, sin(u.get(2)) / u.get(2));
            exp.set(0, 1, (cos(u.get(2)) - 1) / u.get(2));
            exp.set(1, 0, (1 - cos(u.get(2))) / u.get(2));
            exp.set(1, 1, sin(u.get(2)) / u.get(2));
        }
        return x.plus(rot.mult(exp.mult(u)));
    }
    @Override
    public SimpleMatrix F(SimpleMatrix x, SimpleMatrix u) {
        return SimpleMatrix.identity(3);
    }
    @Override
    public SimpleMatrix Q(SimpleMatrix x) {
        return Q;
    }
    @Override
    public Pose pos() {
        return new Pose(state());
    }
    @Override
    public Pose vel() {
        return vel;
    }
    @Override
    public void setPose(Pose p) {
        set(new SimpleMatrix(new double[] {p.x, p.y, p.h}), new SimpleMatrix(3, 3));
    }
    public void setPose(Pose p, double time) {
        set(new SimpleMatrix(new double[] {p.x, p.y, p.h}), new SimpleMatrix(3, 3), time);
    }
    @Override
    public void update(double time) {
        run(time);
    }
}
