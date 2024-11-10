package org.firstinspires.ftc.teamcode.hardware;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.RepeatCommand;
import org.firstinspires.ftc.teamcode.command.WaitCommand;
import org.firstinspires.ftc.teamcode.control.AsymProfile.AsymConstraints;
import org.firstinspires.ftc.teamcode.control.PidfCoefficients;
import org.firstinspires.ftc.teamcode.movement.AbstractMecanumDrive;

public class MecanumDrive extends AbstractMecanumDrive {
    public static final double trackWidth = 0;
    public static final double driveKv = 0;
    public static final double driveKa = 0;
    public static final double driveKs = 0;
    public static final double strafeMult = 1;
    public static final PidfCoefficients xCoeffs = new PidfCoefficients(0, 0, 0);
    public static final PidfCoefficients yCoeffs = new PidfCoefficients(0, 0, 0);
    public static final PidfCoefficients turnCoeffs = new PidfCoefficients(0, 0, 0);
    public static final AsymConstraints moveConstraints = new AsymConstraints(60, 70, 70);
    public static final AsymConstraints turnConstraints = new AsymConstraints(6, 12, 12);
    private Otos otos;
    private double heading = 0;
    private double offset = 0;
    public MecanumDrive(CommandOpMode opMode, boolean auto) {
        super(trackWidth, driveKs, driveKv, driveKa, strafeMult, xCoeffs, yCoeffs, turnCoeffs, moveConstraints, turnConstraints, auto);
        fr = opMode.hardwareMap.get(DcMotorEx.class, "fr");
        fl = opMode.hardwareMap.get(DcMotorEx.class, "fl");
        br = opMode.hardwareMap.get(DcMotorEx.class, "br");
        bl = opMode.hardwareMap.get(DcMotorEx.class, "bl");
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        otos = opMode.hardwareMap.get(Otos.class, "otos");
        otos.setAngularUnit(AngleUnit.RADIANS);
        otos.calibrateImu(255, true);
        //localizer = new OtosLocalizer();
        if (!auto) {
            opMode.schedule(new RepeatCommand(new WaitCommand(t -> heading = otos.getPosition().h, 0.2)));
        }
    }
    public void setHeading(double h) {
        offset = h - heading;
    }
    public double getHeading() {
        return offset + heading;
    }
}