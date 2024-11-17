package org.firstinspires.ftc.teamcode.hardware;
import android.util.Pair;
import org.firstinspires.ftc.teamcode.movement.Localizer;
import org.firstinspires.ftc.teamcode.movement.Pose;
public class OtosLocalizer implements Localizer {
    private Pose pos = new Pose(0, 0, 0);
    private Pose vel = new Pose(0, 0, 0);
    private Otos otos;
    public OtosLocalizer(Otos otos) {
        this.otos = otos;
    }
    @Override
    public Pose pos() {
        return pos;
    }
    @Override
    public Pose vel() {
        return vel;
    }
    @Override
    public void setPose(Pose p) {
        pos = p;
        vel = new Pose(0, 0, 0);
        otos.setPosition(p);
    }
    @Override
    public void update(double time) {
        Pair<Pose, Pose> posVel = otos.getPosVel();
        pos = posVel.first;
        vel = posVel.second;
    }
}