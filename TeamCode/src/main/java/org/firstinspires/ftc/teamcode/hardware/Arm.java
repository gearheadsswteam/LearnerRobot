package org.firstinspires.ftc.teamcode.hardware;
import static java.lang.Math.PI;
import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.FnCommand;
import org.firstinspires.ftc.teamcode.command.Subsystem;
public class Arm implements Subsystem {
    public static class ArmPosition {
        public static final double armZero = 0.58;
        public static final double armRange = 3.16;
        public static final double diffZero = 0.375;
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
    public static final double armUp = 0.35;
    public static final ArmPosition armRest = new ArmPosition(0, 0, 0);
    public static final ArmPosition armGrab = new ArmPosition(armUp, 0, 0);
    public static final ArmPosition armBucket = new ArmPosition(armUp, 1.10, 0);
    public static final ArmPosition armSideChamber = new ArmPosition(0, 0, -1.31);
    public static final ArmPosition armBackChamber = new ArmPosition(armUp, 2.79, 0);
    public static final ArmPosition armGrabbed = new ArmPosition(armUp, 1.89, 0);
    public static final double clawOpen = 0.08;
    public static final double clawClosed = 0.50;
    public static final double grabHyst = 0.262;
    private ServoImplEx arm;
    private ServoImplEx diffR;
    private ServoImplEx diffL;
    private Servo claw;
    private ArmPosition armPos = armGrab;
    private boolean wristFlipped = false;
    public Arm(CommandOpMode opMode, boolean auto) {
        arm = opMode.hardwareMap.get(ServoImplEx.class, "arm");
        diffR = opMode.hardwareMap.get(ServoImplEx.class, "diffR");
        diffL = opMode.hardwareMap.get(ServoImplEx.class, "diffL");
        claw = opMode.hardwareMap.get(Servo.class, "claw");
        arm.setPwmRange(new PwmControl.PwmRange(500, 2500));
        diffR.setPwmRange(new PwmControl.PwmRange(500, 2500));
        diffL.setPwmRange(new PwmControl.PwmRange(500, 2500));
        if (auto) {
            armPos = armBucket;
            opMode.schedule(FnCommand.once(t -> setClaw(true)));
        } else {
            armPos = armGrab;
            setClaw(false);
        }
    }
    public void setArm(ArmPosition pos) {
        armPos = pos;
    }
    public void setClaw(boolean closed) {
        claw.setPosition(closed ? clawClosed : clawOpen);
    }
    public ArmPosition armPos() {
        return armPos;
    }
    public void setGrab(double angle, Lift lift) {
        double relAng = angle - lift.liftPos(lift.restTime()).turretAng;
        double rot = (relAng % (2*PI) + 3*PI) % (2*PI) - PI;
        double flippedRot = ((relAng + PI) % (2*PI) + 3*PI) % (2*PI) - PI;
        if ((!wristFlipped && abs(rot) < PI/2 + grabHyst) ||
                (wristFlipped && abs(flippedRot) > PI/2 + grabHyst)) {
            wristFlipped = false;
            armPos = new ArmPosition(armPos.armAng, armPos.wristAng, rot);
        } else {
            wristFlipped = true;
            armPos = new ArmPosition(armPos.armAng, armPos.wristAng, flippedRot);
        }
    }
    public void update(double t, boolean active) {
        arm.setPosition(armPos.armPos);
        diffR.setPosition(armPos.diffRPos);
        diffL.setPosition(armPos.diffLPos);
    }
}
