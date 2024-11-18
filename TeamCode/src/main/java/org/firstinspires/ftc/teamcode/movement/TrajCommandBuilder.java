package org.firstinspires.ftc.teamcode.movement;
import static java.lang.Double.NaN;
import static java.lang.Double.isNaN;
import static java.lang.Math.*;
import org.firstinspires.ftc.teamcode.command.Command;
import org.firstinspires.ftc.teamcode.command.FnCommand;
import org.firstinspires.ftc.teamcode.command.Scheduler;
import org.firstinspires.ftc.teamcode.control.AsymProfile.AsymConstraints;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleConsumer;

public class TrajCommandBuilder {
    private Drivetrain drive;
    private Pose pos;
    private Vec tangent;
    private double vi;
    private double vf;
    private AsymConstraints moveConstraints;
    private AsymConstraints turnConstraints;
    private boolean compound;
    private List<Path> paths = new ArrayList<>();
    private List<Double> headings = new ArrayList<>();
    private List<List<Double>> scales = new ArrayList<>();
    private List<List<Double>> offsets = new ArrayList<>();
    private List<Trajectory> trajs = new ArrayList<>();
    private List<Double> times = new ArrayList<>(Arrays.asList(0d));
    private List<List<Command>> markers = new ArrayList<>();
    private List<List<Double>> mTimes = new ArrayList<>();
    private List<Command> globalMarkers = new ArrayList<>();
    private List<Double> globalScales = new ArrayList<>();
    private List<Double> globalOffsets = new ArrayList<>();
    public TrajCommandBuilder(Drivetrain drive, Pose pos) {
        this.drive = drive;
        this.pos = pos;
        tangent = Vec.dir(pos.h);
        headings.add(pos.h);
        moveConstraints = drive.getMoveConstraints();
        turnConstraints = drive.getTurnConstraints();
    }
    public TrajCommandBuilder(Drivetrain drive, Pose pos, Vec vel) {
        this.drive = drive;
        this.pos = pos;
        tangent = vel.normalize();
        headings.add(pos.h);
        vi = vel.norm();
        moveConstraints = drive.getMoveConstraints();
        turnConstraints = drive.getTurnConstraints();
    }
    public TrajCommandBuilder setTangent(double h) {
        tangent = Vec.dir(h);
        return this;
    }
    public TrajCommandBuilder setVel(double v) {
        vf = v;
        return this;
    }
    public TrajCommandBuilder setMoveConstraints(AsymConstraints moveConstraints) {
        this.moveConstraints = moveConstraints;
        return this;
    }
    public TrajCommandBuilder setTurnConstraints(AsymConstraints turnConstraints) {
        this.turnConstraints = turnConstraints;
        return this;
    }
    public TrajCommandBuilder resetConstraints() {
        moveConstraints = drive.getMoveConstraints();
        turnConstraints = drive.getTurnConstraints();
        return this;
    }
    public TrajCommandBuilder pause(double t) {
        if (isNaN(vf)) {
            throw new IllegalStateException("Compound path must be closed");
        }
        compound = false;
        pos = new Pose(pos.vec().combo(1, tangent, vi * t), pos.h);
        vf = 0;
        times.add(times.get(times.size() - 1) + t);
        trajs.add(new WaitTrajectory(pos, tangent.mult(vi), t));
        markers.add(new ArrayList<>());
        mTimes.add(new ArrayList<>());
        return this;
    }
    public TrajCommandBuilder turn(double h) {
        if (isNaN(vf)) {
            throw new IllegalStateException("Compound path must be closed");
        }
        if (vi != 0) {
            throw new IllegalArgumentException("Must be stationary to turn");
        }
        compound = false;
        Trajectory traj = new TurnTrajectory(turnConstraints, pos, h);
        pos = new Pose(pos.vec(), h);
        tangent = Vec.dir(pos.h);
        headings = new ArrayList<>(Arrays.asList(pos.h));
        vf = 0;
        times.add(times.get(times.size() - 1) + traj.tf());
        trajs.add(traj);
        markers.add(new ArrayList<>());
        mTimes.add(new ArrayList<>());
        return this;
    }
    public TrajCommandBuilder lineTo(Pose end) {
        addTraj(new LinePath(pos.vec(), end.vec()), end.h);
        return this;
    }
    public TrajCommandBuilder lineTo(Vec end) {
        addTraj(new LinePath(pos.vec(), end), NaN);
        return this;
    }
    public TrajCommandBuilder lineToX(double x, double h) {
        addTraj(LinePath.extendX(pos.vec(), tangent, x), h);
        return this;
    }
    public TrajCommandBuilder lineToX(double x) {
        addTraj(LinePath.extendX(pos.vec(), tangent, x), NaN);
        return this;
    }
    public TrajCommandBuilder lineToY(double y, double h) {
        addTraj(LinePath.extendY(pos.vec(), tangent, y), h);
        return this;
    }
    public TrajCommandBuilder lineToY(double y) {
        addTraj(LinePath.extendY(pos.vec(), tangent, y), NaN);
        return this;
    }
    public TrajCommandBuilder forward(double d) {
        lineTo(pos.vec().combo(1, Vec.dir(pos.h), d));
        return this;
    }
    public TrajCommandBuilder back(double d) {
        lineTo(pos.vec().combo(1, Vec.dir(pos.h + PI), d));
        return this;
    }
    public TrajCommandBuilder right(double d) {
        lineTo(pos.vec().combo(1, Vec.dir(pos.h - PI / 2), d));
        return this;
    }
    public TrajCommandBuilder left(double d) {
        lineTo(pos.vec().combo(1, Vec.dir(pos.h + PI / 2), d));
        return this;
    }
    public TrajCommandBuilder splineTo(Pose end, double t) {
        return splineTo(end, t, end.vec().combo(1, pos.vec(), -1).norm());
    }
    public TrajCommandBuilder splineTo(Vec end, double t) {
        return splineTo(end, t, end.combo(1, pos.vec(), -1).norm());
    }
    public TrajCommandBuilder splineTo(Pose end, double t, double v) {
        addTraj(new SplinePath(pos.vec(), tangent.mult(v), end.vec(), Vec.dir(t).mult(v)), end.h);
        return this;
    }
    public TrajCommandBuilder splineTo(Vec end, double t, double v) {
        addTraj(new SplinePath(pos.vec(), tangent.mult(v), end, Vec.dir(t).mult(v)), NaN);
        return this;
    }
    public TrajCommandBuilder marker(DoubleConsumer fn) {
        return marker(FnCommand.once(fn));
    }
    public TrajCommandBuilder marker(double scale, double offset, DoubleConsumer fn) {
        return marker(scale, offset, FnCommand.once(fn));
    }
    public TrajCommandBuilder marker(Command command) {
        return marker(0, 0, command);
    }
    public TrajCommandBuilder marker(double scale, double offset, Command command) {
        if (trajs.isEmpty()) {
            globalMarkers.add(command);
            globalScales.add(scale);
            globalOffsets.add(offset);
        } else if (compound) {
            markers.get(markers.size() - 1).add(command);
            scales.get(scales.size() - 1).add(scale);
            offsets.get(offsets.size() - 1).add(offset);
        } else {
            double[] tfs = trajs.get(trajs.size() - 1).tfs();
            markers.get(markers.size() - 1).add(command);
            mTimes.get(mTimes.size() - 1).add(times.get(times.size() - 2) + tfs[tfs.length - 2] +
                    (tfs[tfs.length - 1] - tfs[tfs.length - 2]) * scale + offset);
        }
        return this;
    }
    private void addTraj(Path p, double h) {
        if (paths.isEmpty()) {
            markers.add(new ArrayList<>());
        }
        paths.add(p);
        headings.add(h);
        pos = new Pose(p.state(1).pos, isNaN(h) ? pos.h - p.state(0).dir.angle() + p.state(1).dir.angle() : h);
        tangent = vf == 0 ? Vec.dir(pos.h) : p.state(1).dir;
        if (isNaN(vf)) {
            compound = true;
            scales.add(new ArrayList<>());
            offsets.add(new ArrayList<>());
        } else {
            compound = false;
            PathTrajectory traj = new PathTrajectory(paths.toArray(new Path[0]), headings.stream().mapToDouble(i -> i).toArray(),
                    moveConstraints, turnConstraints, vi, vf);
            trajs.add(traj);
            mTimes.add(new ArrayList<>());
            for (int i = 0; i < scales.size(); i++) {
                for (int j = 0; j < scales.get(i).size(); j++) {
                    mTimes.get(mTimes.size() - 1).add(times.get(times.size() - 1) + traj.tfs()[i] +
                            (traj.tfs()[i + 1] - traj.tfs()[i]) * scales.get(i).get(j) + offsets.get(i).get(j));
                }
            }
            headings = new ArrayList<>(Arrays.asList(pos.h));
            paths.clear();
            scales.clear();
            offsets.clear();
            times.add(times.get(times.size() - 1) + traj.tf());
            vi = vf;
        }
        vf = 0;
    }
    public Command build(Scheduler scheduler) {
        for (int i = 0; i < globalMarkers.size(); i++) {
            markers.get(0).add(globalMarkers.get(i));
            mTimes.get(0).add(globalScales.get(i) * times.get(times.size() - 1) + globalOffsets.get(i));
        }
        return new Command(drive) {
            private int index;
            private double ti;
            private Map<Command, Double> currMarkers;
            private ArrayList<Command> removed = new ArrayList<>();
            @Override
            public void init(double time) {
                index = 0;
                ti = time;
                trajs.get(0).setTi(ti);
                drive.setTrajectory(trajs.get(0));
                currMarkers = new HashMap<>();
                for (int i = 0; i < markers.get(0).size(); i++) {
                    currMarkers.put(markers.get(0).get(i), mTimes.get(0).get(i) + ti);
                }
            }
            @Override
            public void run(double time) {
                if (index != trajs.size() - 1 && time > ti + times.get(index + 1)) {
                    index++;
                    trajs.get(index).setTi(times.get(index) + ti);
                    drive.setTrajectory(trajs.get(index));
                    for (int i = 0; i < markers.get(index).size(); i++) {
                        currMarkers.put(markers.get(index).get(i), mTimes.get(index).get(i) + ti);
                    }
                }
                for (Map.Entry<Command, Double> p : currMarkers.entrySet()) {
                    if (time > p.getValue()) {
                        scheduler.schedule(p.getKey());
                        removed.add(p.getKey());
                    }
                }
                for (Command command : removed) {
                    currMarkers.remove(command);
                }
                removed.clear();
            }
            @Override
            public void end(double time, boolean canceled) {
                for (Map.Entry<Command, Double> p : currMarkers.entrySet()) {
                    if (time > p.getValue()) {
                        scheduler.schedule(p.getKey());
                    }
                }
            }
            @Override
            public boolean done(double time) {
                return time > ti + times.get(times.size() - 1);
            }
        };
    }
}