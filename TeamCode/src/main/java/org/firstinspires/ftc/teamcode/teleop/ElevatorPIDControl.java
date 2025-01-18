package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@Config  // Allows FTC Dashboard to expose variables for tuning
@TeleOp(name = "Elevator PID Control (Dashboard)", group = "TeleOp")
public class ElevatorPIDControl extends LinearOpMode {

    // Declare elevator motors
    private DcMotor leftElevatorMotor;
    private DcMotor rightElevatorMotor;

    // Encoder ticks per inch (adjust based on your elevator hardware)
    private static final double TICKS_PER_REV = 1872; // Assuming GoBilda 5203 with 5:2:1 gear ratio motor
    private static final double SPOOL_DIAMETER_INCHES = 4; // Example diameter
    private static final double TICKS_PER_INCH = (TICKS_PER_REV / (Math.PI * SPOOL_DIAMETER_INCHES));

    // PID coefficients (now exposed to FTC Dashboard for tuning)
    public static double Kp = 0.01;  // Public and static for dashboard use
    public static double Ki = 0.0;
    public static double Kd = 0.002;

    // PID variables
    private double integralSum = 0;
    private double lastError = 0;

    private FtcDashboard dashboard;

    @Override
    public void runOpMode() {

        // Initialize motors
        leftElevatorMotor = hardwareMap.get(DcMotor.class, "leftElevatorMotor");
        rightElevatorMotor = hardwareMap.get(DcMotor.class, "rightElevatorMotor");

        // Set motor directions (adjust as needed)
        leftElevatorMotor.setDirection(DcMotor.Direction.REVERSE);
        rightElevatorMotor.setDirection(DcMotor.Direction.FORWARD);

        // Reset encoders
        leftElevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightElevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftElevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightElevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        dashboard = FtcDashboard.getInstance();  // Initialize the dashboard

        waitForStart();

        // Target height in inches
        double targetHeightInches = 0;

        while (opModeIsActive()) {
            // Gamepad control to set elevator height (increment in inches)
            if (gamepad1.dpad_up) {
                targetHeightInches += 1.0;
            } else if (gamepad1.dpad_down) {
                targetHeightInches -= 1.0;
            }

            targetHeightInches = Math.max(0, targetHeightInches); // Prevent going below zero inches

            // PID control
            double power = getPIDPower(targetHeightInches);
            leftElevatorMotor.setPower(power);
            rightElevatorMotor.setPower(power);

            // Send telemetry data to FTC Dashboard
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Target Height (inches)", targetHeightInches);
            packet.put("Current Height (inches)", getCurrentHeightInches());
            packet.put("Motor Power", power);
            packet.put("Error", targetHeightInches - getCurrentHeightInches());
            dashboard.sendTelemetryPacket(packet);  // Send telemetry to dashboard

            telemetry.addData("Target Height (inches)", targetHeightInches);
            telemetry.addData("Current Height (inches)", getCurrentHeightInches());
            telemetry.addData("Motor Power", power);
            telemetry.update();
        }
    }

    private double getCurrentHeightInches() {
        int currentPosition = (leftElevatorMotor.getCurrentPosition() + rightElevatorMotor.getCurrentPosition()) / 2;
        return currentPosition / TICKS_PER_INCH;
    }

    private double getPIDPower(double targetHeightInches) {
        double currentHeightInches = getCurrentHeightInches();
        double error = targetHeightInches - currentHeightInches;

        integralSum += error;
        double derivative = error - lastError;
        lastError = error;

        double power = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

        // Clip power to valid range (-1 to 1)
        return Math.max(-1, Math.min(power, 1));
    }
}
