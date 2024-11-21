package org.firstinspires.ftc.teamcode.hardware;
import static org.firstinspires.ftc.teamcode.hardware.Lift.*;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.StateMachine;
public class Robot {
    public final MecanumDrive drive;
    public final Intake intake;
    public final Lift lift;
    public final Arm arm;
    public StateMachine<RobotStateMachine.robotStates> stateMachine;
    public Robot(CommandOpMode opMode, boolean auto) {
        drive = new MecanumDrive(opMode, auto);
        intake = new Intake(opMode, auto);
        lift = new Lift(drive, opMode, auto);
        arm = new Arm(opMode, auto);
        if (auto) {
            stateMachine = RobotStateMachine.get(opMode, this, RobotStateMachine.robotStates.GRABBED);
        } else {
            stateMachine = RobotStateMachine.get(opMode, this, RobotStateMachine.robotStates.INTAKE);
        }
    }
}