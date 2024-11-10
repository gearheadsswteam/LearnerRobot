package org.firstinspires.ftc.teamcode.movement;
import org.ejml.simple.SimpleMatrix;
public class Pose {
    public final double x;
    public final double y;
    public final double h;
    public Pose(double x, double y, double h) {
        this.x = x;
        this.y = y;
        this.h = h;
    }
    public Pose(Vec v, double h) {
        this(v.x, v.y, h);
    }
    public Pose(SimpleMatrix m) {
        x = m.get(0);
        y = m.get(1);
        h = m.get(2);
    }
    public Vec vec() {
        return new Vec(x, y);
    }
    public boolean zero() {
        return x == 0 && y == 0 && h == 0;
    }
}
