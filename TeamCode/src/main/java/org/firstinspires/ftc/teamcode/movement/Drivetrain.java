package org.firstinspires.ftc.teamcode.movement;
import org.firstinspires.ftc.teamcode.command.Subsystem;
import org.firstinspires.ftc.teamcode.control.AsymProfile.AsymConstraints;
public abstract class Drivetrain implements Subsystem {
    private AsymConstraints moveConstraints;
    private AsymConstraints turnConstraints;
    protected Trajectory traj;
    protected Localizer localizer;
    private boolean auto;
    public Drivetrain(boolean auto, AsymConstraints moveConstraints, AsymConstraints turnConstraints) {
        this.auto = auto;
        this.moveConstraints = moveConstraints;
        this.turnConstraints = turnConstraints;
    }
    public Pose pose() {
        return localizer.pos();
    }
    public Pose vel() {
        return localizer.vel();
    }
    public void setPose(Pose p) {
        localizer.setPose(p);
    }
    public Trajectory getTrajectory() {
        return traj;
    }
    public void setTrajectory(Trajectory traj) {
        this.traj = traj;
    }
    public AsymConstraints getMoveConstraints() {
        return moveConstraints;
    }
    public void setMoveConstraints(AsymConstraints c) {
        moveConstraints = c;
    }
    public AsymConstraints getTurnConstraints() {
        return turnConstraints;
    }
    public void setTurnConstraints(AsymConstraints c) {
        turnConstraints = c;
    }
    public abstract void follow(double time);
    public void updateDrivetrain(double time, boolean active) {}
    @Override
    public void update(double time, boolean active) {
        updateDrivetrain(time, active);
        if (active && auto && localizer != null) {
            localizer.update(time);
            if (traj != null) {
                follow(time);
            }
        }
    }
}