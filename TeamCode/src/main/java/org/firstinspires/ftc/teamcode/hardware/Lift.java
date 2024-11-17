package org.firstinspires.ftc.teamcode.hardware;
import static java.lang.Math.*;
import static com.qualcomm.robotcore.util.Range.*;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.command.Command;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.FnCommand;
import org.firstinspires.ftc.teamcode.command.Subsystem;
import org.firstinspires.ftc.teamcode.control.AsymProfile;
import org.firstinspires.ftc.teamcode.control.AsymProfile.AsymConstraints;
import org.firstinspires.ftc.teamcode.control.ChainProfile;
import org.firstinspires.ftc.teamcode.control.DelayProfile;
import org.firstinspires.ftc.teamcode.control.MotionProfile;
import org.firstinspires.ftc.teamcode.control.MotionState;
import org.firstinspires.ftc.teamcode.control.PidfCoefficients;
import org.firstinspires.ftc.teamcode.control.PidfController;
import org.firstinspires.ftc.teamcode.control.RampProfile;
import org.firstinspires.ftc.teamcode.movement.Vec;

@Config
public class Lift implements Subsystem {
    public static class LiftPosition {
        public static final double liftInToTicks = 62.9;
        public static final double turretRadToTicks = 168.4;
        public static final double pivotRadToTicks = 478.6;
        public static final double pivotRadToLiftIn = 0.32;
        public static final double armLen = 5.91;
        public final double liftExt;
        public final double turretAng;
        public final double pivotAng;
        public LiftPosition(double liftExt, double turretAng, double pivotAng) {
            this.liftExt = liftExt  ;
            this.turretAng = turretAng;
            this.pivotAng = pivotAng;
        }
        public static LiftPosition inverse(Vec v) {
            double turretAng = asin(v.y / armLen);
            double liftExt = v.x + armLen * (1 - cos(turretAng));
            return new LiftPosition(liftExt, turretAng, 0);
        }
        public Vec forwards() {
            return new Vec(liftExt + armLen * (cos(turretAng) - 1), armLen * sin(turretAng));
        }
        public static LiftPosition fromPos(double pivotTicks, double liftRTicks, double liftLTicks) {
            double pivotAng = pivotTicks / pivotRadToTicks;
            double turretAng = (liftRTicks - liftLTicks) / turretRadToTicks;
            double liftExt = (liftRTicks + liftLTicks) / liftInToTicks + pivotAng * pivotRadToLiftIn;
            return new LiftPosition(liftExt, turretAng, pivotAng);
        }
    }
    public static final double liftXMin = 5.5;
    public static final double liftXMax = 24;
    public static final double liftYMax = 5.5;
    public static final double pivotUp = 1.71;
    public static final LiftPosition liftSpecimen = new LiftPosition(8, 0, 0);
    public static final LiftPosition liftHighBucket = new LiftPosition(30.5, 0, pivotUp);
    public static final LiftPosition liftLowBucket = new LiftPosition (15, 0, pivotUp);
    public static final LiftPosition liftLowSideChamber = new LiftPosition(6, PI/2, pivotUp);
    public static final LiftPosition liftHighSideChamber = new LiftPosition(19, PI/2, pivotUp);
    public static final LiftPosition liftHighBackChamber = new LiftPosition(13, 0, pivotUp);
    public static double pivotKp = 5;
    public static double pivotKi = 0;
    public static double pivotKd = 0;
    public static double pivotKgs = 0.02;
    public static double pivotKgd = 0.013;
    public static double pivotKv = 0.15;
    public static double pivotKa = 0.01;
    public static final PidfCoefficients pivotCoeffs = new PidfCoefficients(
        pivotKp, pivotKi, pivotKd, a -> {
            MotionState pivotState = (MotionState)a[0];
            MotionState liftState = (MotionState)a[1];
            return (pivotState.x > pivotUp - 1e-4) ? 0.25 : (pivotKgs + pivotKgd * liftState.x) * cos(pivotState.x)
                    + pivotKv * pivotState.v + pivotKa * pivotState.a;});
    public static double liftKp = 0.5;
    public static double liftKi = 0.5;
    public static double liftKd = 0;
    public static double liftKgs = 0.04;
    public static double liftKgd = 0.001;
    public static double liftKs = 0.1;
    public static double liftKv = 0.012;
    public static double liftKa = 0.001;
    public static final PidfCoefficients liftCoeffs = new PidfCoefficients(
        liftKp, liftKi, liftKd, a -> {
            MotionState pivotState = (MotionState)a[0];
            MotionState liftState = (MotionState)a[1];
            return (liftKgs + liftKgd * liftState.x) * sin(pivotState.x) +
                    liftKs * signum(liftState.v) + liftKv * liftState.v + liftKa * liftState.a;});
    public static double turretKp = 2;
    public static double turretKi = 2;
    public static double turretKd = 0;
    public static double turretKs = 0.1;
    public static double turretKv = 0.03;
    public static double turretKa = 0.001;
    public static final PidfCoefficients turretCoeffs = new PidfCoefficients(
        turretKp, turretKi, turretKd, a -> {
            MotionState turretState = (MotionState)a[0];
            return turretKs * signum(turretState.v) + turretKv * turretState.v + turretKa * turretState.a;});
    public static double pivotVm = 5;
    public static double pivotAi = 50;
    public static double pivotAf = 20;
    public static final AsymConstraints pivotConstraints = new AsymConstraints(pivotVm, pivotAi, pivotAf);
    public static final AsymConstraints pivotBackConstraints = new AsymConstraints(2, 15, 15);
    public static double liftVm = 75;
    public static double liftAi = 750;
    public static double liftAf = 200;
    public static final AsymConstraints liftConstraints = new AsymConstraints(liftVm, liftAi, liftAf);
    public static double turretVm = 20;
    public static double turretAi = 100;
    public static double turretAf = 50;
    public static final AsymConstraints turretConstraints = new AsymConstraints(turretVm, turretAi, turretAf);
    private double adjustV = 12;
    private DcMotorEx pivot;
    private DcMotorEx liftR;
    private DcMotorEx liftL;
    private PidfController pivotPidf = new PidfController(pivotCoeffs);
    private PidfController liftPidf = new PidfController(liftCoeffs);
    private PidfController turretPidf = new PidfController(turretCoeffs);
    private MotionProfile pivotProfile;
    private MotionProfile liftProfile;
    private MotionProfile turretProfile;
    private static double pivotOffset = 0;
    private static double liftROffset = 0;
    private static double liftLOffset = 0;
    private double zeroTime;
    public Lift(CommandOpMode opMode, boolean auto) {
        pivot = opMode.hardwareMap.get(DcMotorEx.class, "pivot");
        liftR = opMode.hardwareMap.get(DcMotorEx.class, "liftR");
        liftL = opMode.hardwareMap.get(DcMotorEx.class, "liftL");
        liftL.setDirection(DcMotorSimple.Direction.REVERSE);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        if (auto) {
            reset(pivotUp);
            zeroTime = Double.NaN;
            pivotProfile = new DelayProfile(0, new MotionState(pivotUp, 0), 0);
            liftProfile = new DelayProfile(0, new MotionState(0, 0), 0);
            turretProfile = new DelayProfile(0, new MotionState(0, 0), 0);
        } else {
            zeroTime = 0;
            pivotProfile = new DelayProfile(0, new MotionState(0, 0), 0);
            liftProfile = new DelayProfile(0, new MotionState(0, 0), 0);
            turretProfile = new DelayProfile(0, new MotionState(0, 0), 0);
        }
    }
    public LiftPosition liftPos(double t) {
        return new LiftPosition(liftProfile.state(t).x, turretProfile.state(t).x, pivotProfile.state(t).x);
    }
    public double restTime() {
        return max(pivotProfile.tf(), max(liftProfile.tf(), turretProfile.tf()));
    }
    public void reset(double pivotAng) {
        pivotOffset = pivot.getCurrentPosition() - pivotAng * LiftPosition.pivotRadToTicks;
        liftROffset = liftR.getCurrentPosition();
        liftLOffset = liftL.getCurrentPosition();
    }
    public Command goTo(LiftPosition pos) {
        return new FnCommand(t -> {
            if (!Double.isNaN(zeroTime)) {
                zeroTime = Double.NaN;
                reset(0);
            }
            if (pivotProfile.state(t).x == pos.pivotAng) {
                liftProfile = AsymProfile.extendAsym(liftProfile, liftConstraints,
                        t, new MotionState(pos.liftExt, 0));
                turretProfile = AsymProfile.extendAsym(turretProfile, turretConstraints,
                        liftProfile.tf(), new MotionState(pos.turretAng, 0));
            } else {
                pivotProfile = AsymProfile.extendAsym(pivotProfile, pivotConstraints,
                        t, new MotionState(pos.pivotAng, 0));
                liftProfile = AsymProfile.extendAsym(liftProfile, liftConstraints,
                        t + 0.15, new MotionState(pos.liftExt, 0));
                turretProfile = AsymProfile.extendAsym(turretProfile, turretConstraints,
                        liftProfile.tf(), new MotionState(pos.turretAng, 0));
            }
        }, t -> {}, (t, b) -> {}, t -> t > restTime(), this);
    }
    public Command goBack() {
        return new FnCommand(t -> {
            turretProfile = AsymProfile.extendAsym(turretProfile, turretConstraints,
                    t, new MotionState(0, 0));
            if (pivotProfile.state(t).x == 0) {
                liftProfile = AsymProfile.extendAsym(liftProfile, liftConstraints,
                        turretProfile.tf(), new MotionState(0, 0));
            } else if (pivotProfile.state(t).x > PI / 2) {
                MotionProfile pivotBackProfile = AsymProfile.extendAsym(pivotProfile, pivotBackConstraints,
                        turretProfile.tf(), new MotionState(PI/2, 0));
                liftProfile = AsymProfile.extendAsym(liftProfile, liftConstraints,
                        pivotBackProfile.tf(), new MotionState(0, 0));
                pivotProfile = new ChainProfile(pivotBackProfile, AsymProfile.extendAsym(pivotBackProfile, pivotConstraints,
                        pivotBackProfile.tf() + 0.15, new MotionState(0, 0)));
            } else {
                liftProfile = AsymProfile.extendAsym(liftProfile, liftConstraints,
                        turretProfile.tf(), new MotionState(0, 0));
                pivotProfile = AsymProfile.extendAsym(pivotProfile, pivotConstraints,
                        turretProfile.tf() + 0.15, new MotionState(0, 0));
            }
        }, t -> {}, (t, b) -> zeroTime = restTime(), t -> t > restTime(), this);
    }
    public Command specimen() {
        return new FnCommand(t -> {liftProfile = AsymProfile.extendAsym(liftProfile, liftConstraints,
                t, new MotionState(liftProfile.state(t).x - 4.5, 0));}, t -> {}, (t, b) -> {}, t -> t > restTime() + 0.15, this);
    }
    public Command adjust(Vec v, double dt) {
        return FnCommand.once(t -> {
            LiftPosition pos = liftPos(t);
            Vec p = pos.forwards();
            double xn = clip(p.x + v.x * adjustV * dt, liftXMin, liftXMax);
            double yn = clip(p.y + v.y * adjustV * dt, -liftYMax, liftYMax);
            LiftPosition posN = LiftPosition.inverse(new Vec(xn, yn));
            liftProfile = RampProfile.extendRamp(liftProfile, t, new MotionState(posN.liftExt, 0), dt);
            turretProfile = RampProfile.extendRamp(turretProfile, t, new MotionState(posN.turretAng, 0), dt);
        }, this);
    }
    @Override
    public void update(double t, boolean active) {
        LiftPosition pos = LiftPosition.fromPos(pivot.getCurrentPosition() - pivotOffset,
                liftR.getCurrentPosition() - liftROffset, liftL.getCurrentPosition() - liftLOffset);
        MotionState pivotState = pivotProfile.state(t);
        MotionState liftState = liftProfile.state(t);
        MotionState turretState = turretProfile.state(t);
        if (pivotState.x < 1e-4) {
            pivot.setPower(-0.15);
            pivotPidf.reset(t);
        } else {
            pivotPidf.set(pivotState.x);
            pivotPidf.update(t, pos.pivotAng, pivotState, liftState);
        }
        if (!Double.isNaN(zeroTime)) {
            double power = (t - zeroTime < 0.25) ? -0.5 : -0.25;
            liftR.setPower(power);
            liftL.setPower(power);
            liftPidf.reset(t);
            turretPidf.reset(t);
        } else {
            liftPidf.set(liftState.x);
            turretPidf.set(turretState.x);
            liftPidf.update(t, pos.liftExt, pivotState, liftState);
            turretPidf.update(t, pos.turretAng, turretState);
            pivot.setPower(pivotPidf.get());
            liftR.setPower(liftPidf.get() + turretPidf.get());
            liftL.setPower(liftPidf.get() - turretPidf.get());
        }
    }
}
