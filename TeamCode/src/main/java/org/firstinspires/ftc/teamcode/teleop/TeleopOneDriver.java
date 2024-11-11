package org.firstinspires.ftc.teamcode.teleop;
import static org.firstinspires.ftc.teamcode.hardware.RobotStateMachine.robotStates.*;
import static org.firstinspires.ftc.teamcode.hardware.ValueStorage.*;
import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.hardware.Lift.*;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.FnCommand;
import org.firstinspires.ftc.teamcode.command.RisingEdgeDetector;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.movement.Vec;
@Photon
@TeleOp(name = "OneDriver")
public class TeleopOneDriver extends CommandOpMode {
    private Robot robot;
    @Override
    public void initOpMode() {
        robot = new Robot(this, false);
        robot.drive.setHeading(lastPose.h + (lastSide == Side.BLUE ? PI / 2 : -PI / 2));
        scheduler.addListener(RisingEdgeDetector.listen(() -> gamepad1.ps,
                FnCommand.once(t -> robot.drive.setHeading(0))),
            RisingEdgeDetector.listen(() -> gamepad1.a, FnCommand.once(t -> {
                if (robot.stateMachine.state() == GRABBED || robot.stateMachine.state() == BUCKET) {
                    robot.stateMachine.transition(BUCKET, liftHighBucket);
                }})),
            RisingEdgeDetector.listen(() -> gamepad1.b, FnCommand.once(t -> {
                if (robot.stateMachine.state() == GRABBED || robot.stateMachine.state() == BUCKET) {
                    robot.stateMachine.transition(BUCKET, liftLowBucket);
                }
            })),
            RisingEdgeDetector.listen(() -> gamepad1.x, FnCommand.once(t -> {
                if (robot.stateMachine.state() == GRABBED || robot.stateMachine.state() == CHAMBER) {
                    robot.stateMachine.transition(CHAMBER, liftHighChamber);
                }
            })),
            RisingEdgeDetector.listen(() -> gamepad1.y, FnCommand.once(t -> {
                if (robot.stateMachine.state() == GRABBED || robot.stateMachine.state() == CHAMBER) {
                    robot.stateMachine.transition(CHAMBER, liftLowChamber);
                }
            })),
            RisingEdgeDetector.listen(() -> gamepad1.right_bumper, FnCommand.once(t -> {
                if (robot.stateMachine.state() == INTAKE || robot.stateMachine.state() == EXTEND_GRAB) {
                    robot.stateMachine.transition(GRABBED);
                } else if (robot.stateMachine.state() == EXTEND) {
                    robot.stateMachine.transition(EXTEND_GRAB);
                } else if (robot.stateMachine.state() == GRABBED) {
                    robot.stateMachine.transition(SPECIMEN);
                } else if (robot.stateMachine.state() == BUCKET || robot.stateMachine.state() == CHAMBER || robot.stateMachine.state() == SPECIMEN) {
                    robot.stateMachine.transition(INTAKE);
                }
            })),
            RisingEdgeDetector.listen(() -> gamepad1.left_bumper, FnCommand.once(t -> {
                if (robot.stateMachine.state() == INTAKE || robot.stateMachine.state() == EXTEND_GRAB) {
                    robot.stateMachine.transition(EXTEND);
                } else if (robot.stateMachine.state() == GRABBED) {
                    robot.stateMachine.transition(INTAKE);
                }
            })));
        schedule(FnCommand.repeat(t -> {
            double f = gamepad1.right_trigger > 0.1 ? 0.25 : 1;
            Vec p = new Vec(-gamepad1.left_stick_y * f, -gamepad1.left_stick_x * f).rotate(-robot.drive.getHeading());
            double turn = -gamepad1.right_stick_x * f;
            if (p.norm() + abs(turn) < 0.05) {
                robot.drive.setPowers(new Vec(0, 0), 0);
            } else {
                robot.drive.setPowers(p, turn);
            }
        }, robot.drive));
    }
}