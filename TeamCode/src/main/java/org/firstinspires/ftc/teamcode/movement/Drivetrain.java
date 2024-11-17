package org.firstinspires.ftc.teamcode.movement;
import static java.lang.Math.*;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
    public AsymConstraints getTurnConstraints() {
        return turnConstraints;
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
                TelemetryPacket packet = new TelemetryPacket();
                Canvas canv = packet.fieldOverlay();
                drawRobot(traj.state(time).pos, "blue", canv);
                drawRobot(localizer.pos(), "green", canv);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }
        }
    }
    public void drawRobot(Pose p, String col, Canvas canv) {
        canv.setStrokeWidth(3);
        canv.setStroke(col);
        canv.strokeCircle(p.x, p.y, 9);
        canv.strokeLine(p.x, p.y, p.x + 9 * cos(p.h), p.y + 9 * sin(p.h));
    }
}