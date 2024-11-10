package org.firstinspires.ftc.teamcode.hardware;
import static java.lang.Math.*;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.core.LynxResetMotorEncoderCommand;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.command.Command;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.FnCommand;
import org.firstinspires.ftc.teamcode.command.Subsystem;
import org.firstinspires.ftc.teamcode.control.AsymProfile;
import org.firstinspires.ftc.teamcode.control.AsymProfile.AsymConstraints;
import org.firstinspires.ftc.teamcode.control.DelayProfile;
import org.firstinspires.ftc.teamcode.control.MotionProfile;
import org.firstinspires.ftc.teamcode.control.MotionState;
import org.firstinspires.ftc.teamcode.control.PidfCoefficients;
import org.firstinspires.ftc.teamcode.control.PidfController;

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
        public static LiftPosition invKin(double x, double y) {
            double turretAng = asin(y / armLen);
            double liftExt = x - armLen * cos(turretAng);
            return new LiftPosition(liftExt, turretAng, 0);
        }
        public static LiftPosition fromPos(double pivotTicks, double liftRTicks, double liftLTicks) {
            double pivotAng = pivotTicks / pivotRadToTicks;
            double turretAng = (liftRTicks - liftLTicks) / turretRadToTicks;
            double liftExt = (liftRTicks + liftLTicks) / liftInToTicks + pivotAng * pivotRadToLiftIn;
            return new LiftPosition(liftExt, turretAng, pivotAng);
        }
    }
    public static class ArmPosition {
        public static final double armZero = 0.58;
        public static final double armRange = 3.16;
        public static final double diffZero = 0.27;
        public static final double diffRange = 5.76;
        public final double armAng;
        public final double wristAng;
        public final double wristRot;
        public final double armPos;
        public final double diffRPos;
        public final double diffLPos;
        public ArmPosition(double armAng, double wristAng, double wristRot) {
            this.armAng = armAng;
            this.wristAng = wristAng;
            this.wristRot = wristRot;
            armPos = armAng / armRange + armZero;
            diffRPos = (wristAng + wristRot + armAng) / diffRange + diffZero;
            diffLPos = (wristAng - wristRot + armAng) / diffRange + diffZero;
        }
    }
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
            return (pivotKgs + pivotKgd * liftState.x) * cos(pivotState.x)
                    + pivotKv * pivotState.v + pivotKa * pivotState.a;});
    public static double liftKp = 0.5;
    public static double liftKi = 0.2;
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
    public static double turretKi = 0;
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
    public static double liftVm = 75;
    public static double liftAi = 750;
    public static double liftAf = 200;
    public static final AsymConstraints liftConstraints = new AsymConstraints(liftVm, liftAi, liftAf);
    public static double turretVm = 20;
    public static double turretAi = 100;
    public static double turretAf = 50;
    public static final AsymConstraints turretConstraints = new AsymConstraints(turretVm, turretAi, turretAf);
    public static final double liftXMin = 5.5;
    public static final double liftXMax = 24;
    public static final double pivotUp = 1.71;
    public static final LiftPosition liftExtend = new LiftPosition(15, 0, 0);
    public static final LiftPosition liftSpecimen = new LiftPosition(8, 0, 0);
    public static final LiftPosition liftHighBucket = new LiftPosition(30.5, 0, pivotUp);
    public static final LiftPosition liftLowBucket = new LiftPosition (16, 0, pivotUp);
    public static final double armUp = 0.35;
    public static final ArmPosition armGrab = new ArmPosition(armUp, 0, 0);
    public static final ArmPosition armBucket = new ArmPosition(armUp, PI/2, 0);
    public static final ArmPosition armFlick = new ArmPosition(armUp, 3*PI/4, 0);
    public static final double clawOpen = 0.08;
    public static final double clawClosed = 0.50;
    public static final double grabHyst = 0.175;
    private DcMotorEx pivot;
    private DcMotorEx liftR;
    private DcMotorEx liftL;
    private ServoImplEx arm;
    private ServoImplEx diffR;
    private ServoImplEx diffL;
    private Servo claw;
    private LynxModule exhub;
    private PidfController pivotPidf = new PidfController(pivotCoeffs);
    private PidfController liftPidf = new PidfController(liftCoeffs);
    private PidfController turretPidf = new PidfController(turretCoeffs);
    private MotionProfile pivotProfile = new DelayProfile(0, new MotionState(0, 0), 0);
    private MotionProfile liftProfile = new DelayProfile(0, new MotionState(0, 0), 0);
    private MotionProfile turretProfile = new DelayProfile(0, new MotionState(0, 0), 0);
    private ArmPosition armPos = armGrab;
    private boolean rest = true;
    public Lift(CommandOpMode opMode, boolean auto) {
        exhub = opMode.hardwareMap.get(LynxModule.class, "exhub");
        pivot = opMode.hardwareMap.get(DcMotorEx.class, "pivot");
        liftR = opMode.hardwareMap.get(DcMotorEx.class, "liftR");
        liftL = opMode.hardwareMap.get(DcMotorEx.class, "liftL");
        liftL.setDirection(DcMotorSimple.Direction.REVERSE);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        arm = opMode.hardwareMap.get(ServoImplEx.class, "arm");
        diffR = opMode.hardwareMap.get(ServoImplEx.class, "diffR");
        diffL = opMode.hardwareMap.get(ServoImplEx.class, "diffL");
        claw = opMode.hardwareMap.get(Servo.class, "claw");
        arm.setPwmRange(new PwmControl.PwmRange(500, 2500));
        diffR.setPwmRange(new PwmControl.PwmRange(500, 2500));
        diffL.setPwmRange(new PwmControl.PwmRange(500, 2500));
        if (auto) {
            reset();
        }
    }
    public double restTime() {
        return max(pivotProfile.tf(), max(liftProfile.tf(), turretProfile.tf()));
    }
    public void reset() {
        try {
            new LynxResetMotorEncoderCommand(exhub, pivot.getPortNumber()).send();
            new LynxResetMotorEncoderCommand(exhub, liftR.getPortNumber()).send();
            new LynxResetMotorEncoderCommand(exhub, liftR.getPortNumber()).send();
        } catch (InterruptedException | LynxNackException e) {}
    }
    public void setArm(ArmPosition pos) {
        armPos = pos;
    }
    public void setGrab(double angle, double armAng, double t) {
        double newAng = ((angle - turretProfile.state(t).x) % (2*PI) + 3*PI) % (2*PI) - PI;
        double rot;
        if (newAng > PI/2) {
            if (newAng < PI/2 + grabHyst && abs(newAng - armPos.wristRot) < PI/2) {
                rot = newAng;
            } else {
                rot = newAng - PI;
            }
        } else if (newAng < -PI/2) {
            if (newAng > -PI/2 - grabHyst && abs(newAng - armPos.wristRot) < PI/2) {
                rot = newAng;
            } else {
                rot = newAng + PI;
            }
        } else {
            rot = newAng;
        }
        armPos = new ArmPosition(armAng, 0, rot);
    }
    public Command goTo(LiftPosition pos) {
        return new FnCommand(t -> {
            if (rest) {
                rest = false;
                reset();
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
            if (pivotProfile.state(t).x == 0) {
                turretProfile = AsymProfile.extendAsym(turretProfile, turretConstraints,
                        t, new MotionState(0, 0));
                liftProfile = AsymProfile.extendAsym(liftProfile, liftConstraints,
                        turretProfile.tf(), new MotionState(0, 0));
            } else {
                turretProfile = AsymProfile.extendAsym(turretProfile, turretConstraints,
                        t, new MotionState(0, 0));
                liftProfile = AsymProfile.extendAsym(liftProfile, liftConstraints,
                        turretProfile.tf(), new MotionState(0, 0));
                pivotProfile = AsymProfile.extendAsym(pivotProfile, pivotConstraints,
                        liftProfile.ti() + 0.15, new MotionState(0, 0));
            }
        }, t -> {}, (t, b) -> rest = true, t -> t > restTime(), this);
    }
    public void setClaw(double pos) {
        claw.setPosition(pos);
    }
    public ArmPosition armPos() {
        return armPos;
    }
    @Override
    public void update(double t, boolean active) {
        LiftPosition pos = LiftPosition.fromPos(pivot.getCurrentPosition(),
                liftR.getCurrentPosition(), liftL.getCurrentPosition());
        MotionState pivotState = pivotProfile.state(t);
        MotionState liftState = liftProfile.state(t);
        MotionState turretState = turretProfile.state(t);
        if (pivotState.x == 0) {
            pivot.setPower(-0.1);
        } else {
            pivotPidf.set(pivotState.x);
            pivotPidf.update(t, pos.pivotAng, pivotState, liftState);
        }
        if (rest) {
            liftR.setPower(-0.3);
            liftL.setPower(-0.3);
        } else {
            liftPidf.set(liftState.x);
            turretPidf.set(turretState.x);
            liftPidf.update(t, pos.liftExt, pivotState, liftState);
            turretPidf.update(t, pos.turretAng, turretState);
            pivot.setPower(pivotPidf.get());
            liftR.setPower(liftPidf.get() + turretPidf.get());
            liftL.setPower(liftPidf.get() - turretPidf.get());
            arm.setPosition(armPos.armPos);
            diffR.setPosition(armPos.diffRPos);
            diffL.setPosition(armPos.diffLPos);
        }
    }
}
