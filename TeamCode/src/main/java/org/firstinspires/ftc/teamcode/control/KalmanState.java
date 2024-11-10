package org.firstinspires.ftc.teamcode.control;
import org.ejml.simple.SimpleMatrix;
import java.util.ArrayList;
import java.util.List;
public class KalmanState {
    public SimpleMatrix x;
    public SimpleMatrix P;
    public SimpleMatrix u;
    public final double time;
    public final List<KalmanEstimate> ests = new ArrayList<>();
    public KalmanState(SimpleMatrix x, SimpleMatrix u, SimpleMatrix P, double time) {
        this.x = x;
        this.u = u;
        this.P = P;
        this.time = time;
    }
}
