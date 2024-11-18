package org.firstinspires.ftc.teamcode.autonomous;
import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.hardware.RobotStateMachine.robotStates.*;
import static org.firstinspires.ftc.teamcode.hardware.Lift.*;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.command.Command;
import org.firstinspires.ftc.teamcode.command.FnCommand;
import org.firstinspires.ftc.teamcode.command.RepeatCommand;
import org.firstinspires.ftc.teamcode.command.SeqCommand;
import org.firstinspires.ftc.teamcode.command.WaitCommand;
import org.firstinspires.ftc.teamcode.control.AsymProfile.AsymConstraints;
import org.firstinspires.ftc.teamcode.movement.Pose;
import org.firstinspires.ftc.teamcode.movement.TrajCommandBuilder;
import org.firstinspires.ftc.teamcode.movement.Vec;

@Photon
@Autonomous(name = "Chamber")
public class Chamber extends AbstractAutonomous {
    private AsymConstraints intakeConstraints = new AsymConstraints(10, 30, 30);
    private AsymConstraints slowConstraints = new AsymConstraints(40, 50, 50);
    private Pose start = new Pose(-6.5, 63, PI/2);
    private Pose specimen1 = new Pose(-5, 31, PI/2);
    private Pose specimen2 = new Pose(-4, 31, 5*PI/6);
    private Pose preIntake = new Pose(-36, 54, 5*PI/6);
    private Pose intake = new Pose(-44, 59, 5*PI/6);
    int i = 0;
    @Override
    public void initAutonomous() {
        Command traj1 = new TrajCommandBuilder(robot.drive, start)
                .lineTo(specimen1)
                .marker(t -> robot.stateMachine.transition(BACK_CHAMBER, liftHighBackChamber))
                .marker(1, -0.15, t -> robot.stateMachine.transition(INTAKE))
                .pause(0.25)
                .setMoveConstraints(slowConstraints)
                .setVel(10)
                .splineTo(preIntake.vec(), preIntake.h)
                .setMoveConstraints(intakeConstraints)
                .lineTo(intake.vec())
                .marker(1, -0.15, t -> robot.stateMachine.transition(GRABBED))
                .build(scheduler);
        Command traj2 = new TrajCommandBuilder(robot.drive, intake)
                .setMoveConstraints(slowConstraints)
                .marker(0, 0.55, t -> robot.stateMachine.transition(SIDE_CHAMBER, liftHighSideChamber))
                .lineTo(specimen2.vec().combo(1, new Vec(2, 0), i))
                .marker(1, -0.15, t -> robot.stateMachine.transition(INTAKE))
                .pause(0.25)
                .setVel(10)
                .lineTo(preIntake.vec())
                .setMoveConstraints(intakeConstraints)
                .lineTo(intake.vec())
                .marker(1, -0.15, t -> {
                    robot.stateMachine.transition(GRABBED);
                    i++;})
                .build(scheduler);
        scheduler.schedule(new SeqCommand(traj1, new RepeatCommand(traj2, 4), new WaitCommand(2, t -> end())));
        robot.drive.setPose(start);
    }
}