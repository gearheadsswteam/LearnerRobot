package org.firstinspires.ftc.teamcode.movement;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
public class Encoder {
    private DcMotorEx motor;
    private double lastTime = Double.NaN;
    int lastPos = 0;
    private Direction dir = Direction.FORWARD;
    public Encoder(DcMotorEx motor) {
        this.motor = motor;
    }
    public void setDirection(Direction direction) {
        dir = direction;
    }
    public Direction getDirection() {
        return dir;
    }
    private int mult() {
        return ((dir == Direction.FORWARD) == (motor.getDirection() == Direction.FORWARD)) ? 1 : -1;
    }
    public int getPosition() {
        return mult() * motor.getCurrentPosition();
    }
    public int getVelocity(double time) {
        if (!Double.isNaN(lastTime)) {
            int vel = (int)motor.getVelocity() & 0xffff;
            double estVel = (lastPos - (lastPos = motor.getCurrentPosition())) / (lastTime - (lastTime = time));
            vel += ((vel % 20) / 4) * 0x10000;
            vel += Math.round((estVel - vel) / (5 * 0x10000)) * 5 * 0x10000;
            return mult() * vel;
        }
        lastPos = motor.getCurrentPosition();
        lastTime = time;
        return mult() * (int)motor.getVelocity();
    }
}