package org.firstinspires.ftc.teamcode;

import android.text.method.Touch;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Mecanum")
public class Mecanum extends LinearOpMode
{
    /* Declare OpMode members. */
    RobotHWMap robot   = new RobotHWMap();   // Use a Pushbot's hardware

    double drive;   // Power for forward and back motion
    double strafe;  // Power for left and right motion
    double rotate;  // Power for rotating the robot

    @Override
    public void runOpMode () throws InterruptedException
    {
        robot.init(hardwareMap);

        robot.motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);  //makes robot go forward
        robot.motorFrontRight.setDirection(DcMotor.Direction.REVERSE); //makes robot go forward
        robot.motorBackLeft.setDirection(DcMotor.Direction.FORWARD);   //makes robot go forward
        robot.motorBackRight.setDirection(DcMotor.Direction.REVERSE);  //makes robot go forward

        waitForStart();

        while(opModeIsActive()) {
//            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
//            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
//            double rightX = gamepad1.right_stick_x;
//            final double v1 = r * Math.cos(robotAngle) - rightX;
//            final double v2 = r * Math.sin(robotAngle) + rightX;
//            final double v3 = r * Math.sin(robotAngle) - rightX;
//            final double v4 = r * Math.cos(robotAngle) + rightX;
//
//            robot.motorFrontLeft.setPower(v1);
//            robot.motorFrontRight.setPower(v2);
//            robot.motorBackLeft.setPower(v3);
//            robot.motorBackRight.setPower(v4);

            drive = gamepad1.left_stick_y;
            strafe = -gamepad1.left_stick_x;  // Negative because the gamepad is weird
            rotate = -gamepad1.right_stick_x; // Negative because the gamepad is weird

            // You might have to play with the + or - depending on how your motors are installed
            robot.motorFrontLeft.setPower(drive + strafe + rotate);
            robot.motorFrontRight.setPower(drive - strafe - rotate);
            robot.motorBackLeft.setPower(drive - strafe + rotate);
            robot.motorBackRight.setPower(drive + strafe - rotate);

            if(gamepad1.left_bumper)
            {
                robot.motorLift.setPower(1);
            }
            else {
                robot.motorLift.setPower(0);
            }
            if(gamepad1.right_bumper)
            {
                robot.motorLift.setPower(-1);
            }
            else {
                robot.motorLift.setPower(0);
            }
        }
    }
}