package org.firstinspires.ftc.teamcode.movement;
public interface Trajectory {
    TrajectoryState state(double t);
    void setTi(double ti);
    double tf();
    double[] tfs();
}
