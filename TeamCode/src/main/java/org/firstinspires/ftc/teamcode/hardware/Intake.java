package org.firstinspires.ftc.teamcode.hardware;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.FnCommand;
import org.firstinspires.ftc.teamcode.command.Subsystem;
public class Intake implements Subsystem {
    private CRServo intakeR;
    private CRServo intakeL;
    public Intake(CommandOpMode opMode, boolean auto) {
        opMode.register(this);
        intakeR = opMode.hardwareMap.get(CRServo.class, "intakeR");
        intakeL = opMode.hardwareMap.get(CRServo.class, "intakeL");
        intakeR.setDirection(DcMotorSimple.Direction.REVERSE);
        if (!auto) {
            opMode.schedule(FnCommand.once(t -> set(0.375), this));
        }
    }
    public void set(double power) {
        intakeR.setPower(power);
        intakeL.setPower(power);
    }
    @Override
    public void update(double t, boolean active) {}
}
