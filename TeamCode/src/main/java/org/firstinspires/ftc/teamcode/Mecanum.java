package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.text.method.Touch;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

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

            // Control arm motors
            //robot.motorLift.setPower(gamepad2.left_stick_y);
            robot.motorPivot.setPower(gamepad2.left_stick_y);
            robot.motorExtend.setPower(gamepad2.right_stick_y);

            /*if(gamepad1.a)
            {
                robot.colorServo.setPosition(0.93);
            }
            if(gamepad1.b)
            {
                robot.colorServo.setPosition(0.60);
            }
            if(gamepad1.x)
            {
                robot.robotClaw.setPosition(1);
            }
            if(gamepad1.y)
            {
                robot.robotClaw.setPosition(0);
            }

            /*

            telemetry.addData("Red", robot.colorSensor.red());
            telemetry.addData("Green", robot.colorSensor.green());
            telemetry.addData("Blue", robot.colorSensor.blue());
            telemetry.addData( "alpha", robot.colorSensor.alpha());
            telemetry.addData( "argb", robot.colorSensor.argb());
            telemetry.update();

            if(robot.colorSensor.blue() > 60 && robot.colorSensor.red() > 60 &&
                    robot.colorSensor.green()> 60)
            {
                telemetry.addLine("White Mineral Found");
                telemetry.update();
            }
            */
            /*
            ColorSensor sensorColor;
            DistanceSensor sensorDistance;
            // hsvValues is an array that will hold the hue, saturation, and value information.
            float hsvValues[] = {0F, 0F, 0F};

            // values is a reference to the hsvValues array.
            final float values[] = hsvValues;

            // sometimes it helps to multiply the raw RGB values with a scale factor
            // to amplify/attentuate the measured values.
            final double SCALE_FACTOR = 255;

            // get a reference to the RelativeLayout so we can change the background
            // color of the Robot Controller app to match the hue detected by the RGB sensor.
            int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
            final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

            // get a reference to the color sensor.
            sensorColor = hardwareMap.get(ColorSensor.class, "colorSensor");

            // get a reference to the distance sensor that shares the same name.
            sensorDistance = hardwareMap.get(DistanceSensor.class, "colorSensor");

            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);

            // send the info back to driver station using telemetry function.
            telemetry.addData("Distance (cm)",
                    String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
            telemetry.addData("Alpha", sensorColor.alpha());
            telemetry.addData("Red  ", sensorColor.red());
            telemetry.addData("Green", sensorColor.green());
            telemetry.addData("Blue ", sensorColor.blue());
            telemetry.addData("Hue", hsvValues[0]);

            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });

            telemetry.update();

        }*/
    }
}}