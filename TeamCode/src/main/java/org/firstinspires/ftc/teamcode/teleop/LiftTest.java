package org.firstinspires.ftc.teamcode.teleop;
import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.hardware.Lift.*;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.core.LynxResetMotorEncoderCommand;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.FnCommand;
import org.firstinspires.ftc.teamcode.command.RisingEdgeDetector;
import org.firstinspires.ftc.teamcode.command.Subsystem;
import org.firstinspires.ftc.teamcode.control.AsymProfile;
import org.firstinspires.ftc.teamcode.control.AsymProfile.AsymConstraints;
import org.firstinspires.ftc.teamcode.control.DelayProfile;
import org.firstinspires.ftc.teamcode.control.MotionProfile;
import org.firstinspires.ftc.teamcode.control.MotionState;
import org.firstinspires.ftc.teamcode.control.PidfCoefficients;
import org.firstinspires.ftc.teamcode.control.PidfController;

@Photon
@Disabled
@TeleOp(name = "LiftTest")
public class LiftTest extends CommandOpMode {
    private LynxModule exhub;
    private DcMotorEx pivot;
    private DcMotorEx liftR;
    private DcMotorEx liftL;
    private PidfCoefficients pivotCoeffs = new PidfCoefficients(
            pivotKp, pivotKi, pivotKd, a -> {
        MotionState pivotState = (MotionState)a[0];
        MotionState liftState = (MotionState)a[1];
        return (pivotKgs + pivotKgd * liftState.x) * cos(pivotState.x)
                + pivotKv * pivotState.v + pivotKa * pivotState.a;});
    private PidfCoefficients liftCoeffs = new PidfCoefficients(
            liftKp, liftKi, liftKd, a -> {
        MotionState pivotState = (MotionState)a[0];
        MotionState liftState = (MotionState)a[1];
        return (liftKgs + liftKgd * liftState.x) * sin(pivotState.x) +
                liftKs * signum(liftState.v) + liftKv * liftState.v + liftKa * liftState.a;});
    private PidfCoefficients turretCoeffs = new PidfCoefficients(
            turretKp, turretKi, turretKd, a -> {
        MotionState turretState = (MotionState)a[0];
        return turretKs * signum(turretState.v) + turretKv * turretState.v + turretKa * turretState.a;});
    private PidfController pivotPidf = new PidfController(pivotCoeffs);
    private PidfController liftPidf = new PidfController(liftCoeffs);
    private PidfController turretPidf = new PidfController(turretCoeffs);
    private MotionProfile pivotProfile = new DelayProfile(0, new MotionState(0), 0);
    private MotionProfile liftProfile = new DelayProfile(0, new MotionState(0), 0);
    private MotionProfile turretProfile = new DelayProfile(0, new MotionState(0), 0);
    private boolean liftOut = false;
    private boolean pivotOut = false;
    @Override
    public void initOpMode() {
        exhub = hardwareMap.get(LynxModule.class, "exhub");
        pivot = hardwareMap.get(DcMotorEx.class, "pivot");
        liftR = hardwareMap.get(DcMotorEx.class, "liftR");
        liftL = hardwareMap.get(DcMotorEx.class, "liftL");
        liftL.setDirection(DcMotorSimple.Direction.REVERSE);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        try {
            new LynxResetMotorEncoderCommand(exhub, pivot.getPortNumber()).send();
            new LynxResetMotorEncoderCommand(exhub, liftR.getPortNumber()).send();
            new LynxResetMotorEncoderCommand(exhub, liftL.getPortNumber()).send();
        } catch (InterruptedException | LynxNackException e) {}
        Subsystem lift = (t, b) -> {
            LiftPosition pos = LiftPosition.fromPos(pivot.getCurrentPosition(),
                    liftR.getCurrentPosition(), liftL.getCurrentPosition());
            MotionState pivotState = pivotProfile.state(t);
            MotionState liftState = liftProfile.state(t);
            MotionState turretState = turretProfile.state(t);
            pivotPidf.set(pivotState.x);
            liftPidf.set(liftState.x);
            turretPidf.set(turretState.x);
            pivotPidf.update(t, pos.pivotAng, pivotState, liftState);
            liftPidf.update(t, pos.liftExt, pivotState, liftState);
            turretPidf.update(t, pos.turretAng, turretState);
            pivot.setPower(pivotPidf.get());
            liftR.setPower(liftPidf.get() + turretPidf.get());
            liftL.setPower(liftPidf.get() - turretPidf.get());
            telemetry.addData("Pivot Angle", pos.pivotAng);
            telemetry.addData("Lift Extension", pos.liftExt);
            telemetry.addData("Turret Angle", pos.turretAng);
            telemetry.addData("Target Pivot Angle", pivotState.x);
            telemetry.addData("Target Lift Extension", liftState.x);
            telemetry.addData("Target Turret Angle", turretState.x);
        };
        register(lift);
        scheduler.addListener(RisingEdgeDetector.listen(() -> gamepad1.a, FnCommand.once(t -> {
                if (t > max(pivotProfile.tf(), liftProfile.tf()) && !pivotOut) {
                    if (liftOut) {
                        liftProfile = AsymProfile.extendAsym(liftProfile,
                                new AsymConstraints(liftVm, liftAi, liftAf), t, new MotionState(0));
                    } else {
                        liftProfile = AsymProfile.extendAsym(liftProfile,
                                new AsymConstraints(liftVm, liftAi, liftAf), t, new MotionState(liftXMax));
                    }
                    liftOut = !liftOut;
                }})),
            RisingEdgeDetector.listen(() -> gamepad1.b, FnCommand.once(t -> {
                if (t > max(pivotProfile.tf(), liftProfile.tf()) && !liftOut) {
                    if (pivotOut) {
                        liftProfile = AsymProfile.extendAsym(liftProfile,
                                new AsymConstraints(liftVm, liftAi, liftAf), t, new MotionState(0));
                        pivotProfile = AsymProfile.extendAsym(pivotProfile,
                                new AsymConstraints(pivotVm, pivotAi, pivotAf), t + 0.15, new MotionState(0));
                    } else {
                        pivotProfile = AsymProfile.extendAsym(pivotProfile,
                                new AsymConstraints(pivotVm, pivotAi, pivotAf), t, new MotionState(liftHighBucket.pivotAng));
                        liftProfile = AsymProfile.extendAsym(liftProfile,
                                new AsymConstraints(liftVm, liftAi, liftAf), t + 0.15, new MotionState(liftHighBucket.liftExt));
                    }
                    pivotOut = !pivotOut;
                }})));
    }
}
