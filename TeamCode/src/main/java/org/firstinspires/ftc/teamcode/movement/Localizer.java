package org.firstinspires.ftc.teamcode.movement;
public interface Localizer {
    Pose pos();
    Pose vel();
    void update(double t);
    void setPose(Pose p);
}