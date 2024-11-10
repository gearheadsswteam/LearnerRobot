package org.firstinspires.ftc.teamcode.movement;
import static java.lang.Math.*;
import java.util.function.DoubleUnaryOperator;
public class AdaptiveQuadrature {
    public static final double[] x = {0, sqrt(5 - 2 * sqrt(10d / 7)) / 3, sqrt(5 + 2 * sqrt(10d / 7)) / 3};
    public static final double[] w = {128d / 225, (322 + 13 * sqrt(70)) / 900, (322 - 13 * sqrt(70)) / 900};
    private DoubleUnaryOperator f;
    public AdaptiveQuadrature(DoubleUnaryOperator f) {
        this.f = f;
    }
    public double integrate(double a, double b, double eps) {
        if (a == b) {
            return 0;
        }
        return adaptive(a, b, eps, quadrature(a, b));
    }
    private double adaptive(double a, double b, double eps, double i) {
        double m = (a + b) / 2;
        double i1 = quadrature(a, m);
        double i2 = quadrature(m, b);
        if (abs(i1 + i2 - i) < eps) {
            return i1 + i2;
        }
        return adaptive(a, m, eps, i1) + adaptive(m, b, eps, i2);
    }
    private double quadrature(double a, double b) {
        double m = (a + b) / 2;
        double d = (b - a) / 2;
        double s = f.applyAsDouble(d * x[0] + m) * w[0];
        for (int i = 1; i < x.length; i++) {
            s += (f.applyAsDouble(d * x[i] + m) + f.applyAsDouble(-d * x[i] + m)) * w[i];
        }
        return d * s;
    }
}