package org.firstinspires.ftc.teamcode.movement;
public interface Path {
    PathState state(double t);
    double length();
}