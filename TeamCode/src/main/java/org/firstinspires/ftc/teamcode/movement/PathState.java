package org.firstinspires.ftc.teamcode.movement;
public class PathState {
    public final Vec pos;
    public final Vec dir;
    public final double curv;
    public PathState(Vec pos, Vec dir, double curv) {
        this.pos = pos;
        this.dir = dir;
        this.curv = curv;
    }
}
