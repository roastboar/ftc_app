package org.firstinspires.ftc.teamcode;

import android.text.method.Touch;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Manual")
public class Manual extends LinearOpMode
{
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorFrontRight;

    @Override
    public void runOpMode () throws InterruptedException
    {
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");

        motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);  //makes robot go forward
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE); //makes robot go forward
        motorBackLeft.setDirection(DcMotor.Direction.FORWARD);   //makes robot go forward
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);  //makes robot go forward

        waitForStart();

        while(opModeIsActive())
        {
            if(gamepad1.left_stick_y == 0)
            {
                motorFrontLeft.setPower(-gamepad1.right_stick_x);
                motorFrontRight.setPower(gamepad1.right_stick_x);
                motorBackLeft.setPower(-gamepad1.right_stick_x);
                motorBackRight.setPower(gamepad1.right_stick_x);
            }
            else {
                motorFrontLeft.setPower(gamepad1.left_stick_y);
                motorFrontRight.setPower(gamepad1.left_stick_y);
                motorBackLeft.setPower(gamepad1.left_stick_y);
                motorBackRight.setPower(gamepad1.left_stick_y);
            }
        }
        idle();
    }
}