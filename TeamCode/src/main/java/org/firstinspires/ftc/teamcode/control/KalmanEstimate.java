package org.firstinspires.ftc.teamcode.control;
import org.ejml.simple.SimpleMatrix;
public class KalmanEstimate {
    public final SimpleMatrix z;
    public final SimpleMatrix H;
    public final SimpleMatrix R;
    public final double time;
    public KalmanEstimate(SimpleMatrix z, SimpleMatrix H, SimpleMatrix R, double time) {
        this.z = z;
        this.H = H;
        this.R = R;
        this.time = time;
    }
}
