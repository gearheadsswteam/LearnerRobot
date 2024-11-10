package org.firstinspires.ftc.teamcode.movement;
public class TrajectoryState {
    public final Pose pos;
    public final Pose vel;
    public final Vec accel;
    public TrajectoryState(Pose pos, Pose vel, Vec accel) {
        this.pos = pos;
        this.vel = vel;
        this.accel = accel;
    }
}
