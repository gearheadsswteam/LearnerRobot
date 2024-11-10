package org.firstinspires.ftc.teamcode.movement;
public interface Localizer {
    Pose pos();
    Pose vel();
    void setPose(Pose p);
    void update(double time);
}