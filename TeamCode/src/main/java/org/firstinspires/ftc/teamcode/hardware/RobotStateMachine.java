package org.firstinspires.ftc.teamcode.hardware;
import static org.firstinspires.ftc.teamcode.hardware.Lift.*;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.FnCommand;
import org.firstinspires.ftc.teamcode.command.ParCommand;
import org.firstinspires.ftc.teamcode.command.SeqCommand;
import org.firstinspires.ftc.teamcode.command.StateMachine;
import org.firstinspires.ftc.teamcode.command.StateMachineBuilder;
import org.firstinspires.ftc.teamcode.command.Subsystem;
import org.firstinspires.ftc.teamcode.command.WaitCommand;

public class RobotStateMachine {
    public enum robotStates {
        INTAKE, EXTEND, EXTEND_GRAB, GRABBED, SPECIMEN, BUCKET, CHAMBER, CLIMB, CLIMBED
    }
    public static StateMachine<robotStates> get(CommandOpMode opMode, Robot robot, robotStates state) {
        Subsystem[] subsystems = {robot.lift, robot.intake};
        StateMachineBuilder<robotStates> builder = new StateMachineBuilder<robotStates>(opMode)
                .addState(robotStates.values())
                .addTransition(robotStates.INTAKE, robotStates.EXTEND, new ParCommand(
                        FnCommand.once(t -> robot.intake.set(0), robot.intake),
                        robot.lift.goTo(liftExtend)))
                .addTransition(robotStates.INTAKE, robotStates.GRABBED, new SeqCommand(
                        new WaitCommand(t -> robot.lift.setArm(armRest), 0.15, t -> robot.lift.setClaw(clawClosed), subsystems),
                        new WaitCommand(0.25, t -> {
                            robot.lift.setArm(armBucket);
                            robot.intake.set(-0.5);}, subsystems),
                        new WaitCommand(0.15, t -> robot.intake.set(0))))
                .addTransition(robotStates.EXTEND, robotStates.EXTEND_GRAB, new WaitCommand(t ->
                        robot.lift.setArm(new ArmPosition(0, 0, robot.lift.armPos().wristRot)), 0.15, robot.lift))
                .addTransition(robotStates.EXTEND_GRAB, robotStates.EXTEND, new WaitCommand(t ->
                        robot.lift.setArm(new ArmPosition(armUp, 0, robot.lift.armPos().wristRot)), 0.15, robot.lift))
                .addTransition(robotStates.EXTEND_GRAB, robotStates.GRABBED, new SeqCommand(
                        new WaitCommand(t -> robot.lift.setClaw(clawClosed), 0.25, subsystems),
                        new WaitCommand(t -> robot.lift.setArm(armBucket), 0.15, subsystems),
                        robot.lift.goBack()))
                .addTransition(robotStates.GRABBED, robotStates.INTAKE, new WaitCommand(t -> {
                    robot.lift.setArm(new ArmPosition(armUp,0, 0));
                    robot.lift.setClaw(clawOpen);
                    robot.intake.set(1);}, 0.25, subsystems))
                .addTransition(robotStates.GRABBED, robotStates.SPECIMEN, new ParCommand(
                        FnCommand.once(t -> robot.intake.set(1), robot.intake),
                        robot.lift.goTo(liftSpecimen)))
                .addTransition(robotStates.SPECIMEN, robotStates.INTAKE, new SeqCommand(
                        new WaitCommand(t -> robot.lift.setClaw(clawOpen), 0.25, t -> robot.lift.setArm(armGrab), robot.lift),
                        robot.lift.goBack()))
                .addTransition(robotStates.GRABBED, robotStates.BUCKET, a -> robot.lift.goTo((LiftPosition)a[0]))
                .addTransition(robotStates.BUCKET, robotStates.BUCKET, a -> robot.lift.goTo((LiftPosition)a[0]))
                .addTransition(robotStates.BUCKET, robotStates.INTAKE, new SeqCommand(
                        new WaitCommand(t -> {
                            robot.lift.setClaw(clawOpen);
                            robot.lift.setArm(armFlick);
                        }, 0.25, subsystems),
                        new WaitCommand(t -> {
                            robot.lift.setArm(armGrab);
                            robot.intake.set(1);}, 0.15, subsystems),
                        robot.lift.goBack()))
                .addTransition(robotStates.GRABBED, robotStates.CHAMBER, a -> new ParCommand(
                    FnCommand.once(t -> robot.lift.setArm(armChamber)), 
                    robot.lift.goTo((LiftPosition)a[0])))
                .addTransition(robotStates.CHAMBER, robotStates.CHAMBER, a -> robot.lift.goTo((LiftPosition)a[0]))
                .addTransition(robotStates.CHAMBER, robotStates.INTAKE, new SeqCommand(
                        robot.lift.specimen(),
                        new WaitCommand(t -> robot.lift.setClaw(clawOpen), 0.25, t -> {
                            robot.lift.setArm(armRest);
                            robot.intake.set(1);}, subsystems),
                        new ParCommand(
                            new WaitCommand(0.25, t -> robot.lift.setArm(armGrab)),
                            robot.lift.goBack())));
        return builder.build(state);
    }
}