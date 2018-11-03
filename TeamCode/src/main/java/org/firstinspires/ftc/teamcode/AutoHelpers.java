package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.lang.Thread;
import java.util.Locale;

public class AutoHelpers
{
    // Encoders measure the amount that the axle on the motor has spun in a unit called 'ticks'.
    // Each model of motor has a slightly different amount of ticks in a full rotation. By knowing
    // the size of your wheel and by tracking the amount of ticks your motor has counted, you can
    // get your distance traveled.

    // For example, if I have a wheel with a diameter of 4 inches, I know that every full rotation
    // of the wheel will cause me to travel a distance of 12.56 inches (circumference=diameter*pi).
    // If my motor has 1440 ticks in a rotation, then each tick is 0.00872664625 inches (12.56/1440)
    // If I want the robot to travel 6 inches, then I tell it to travel for 687 ticks (6/0.0087)

    // Gear reductions just change the amount of ticks per rotation. If I have a 2:1 gear reduction,
    // then it now takes my 1440 tick/rotation motor 2880 ticks to complete a rotation which would
    // then be used above..

    private int InchesToTicks(double inches)
    {
        // Our wheels are 4"
        // One rotation will result in a travel of 12.56 inches.
        // (circumference=diameter*pi)

        // Neverest has 1120 ticks per rotation
        return (int) (inches/(12.56/1120));
    }

    public void ResetEncoders(RobotHWMap robot)
    {
        robot.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /*
        robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        */

        robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void WaitForMotors(RobotHWMap robot)
    {
        while(
                (robot.motorFrontLeft.isBusy() &&
                        robot.motorFrontRight.isBusy() &&
                        robot.motorBackLeft.isBusy() &&
                        robot.motorBackRight.isBusy()
                ))
//                ) && opModeIsActive())
        {
            // nothing
        }
    }

    public void SetMotorPower(RobotHWMap robot, int value)
    {
        robot.motorFrontLeft.setPower(value);
        robot.motorFrontRight.setPower(value);
        robot.motorBackLeft.setPower(value);
        robot.motorBackRight.setPower(value);
    }

    public void StrafeLeft(RobotHWMap Robot, double Inches, long SleepTime)
    {
        int Ticks;

        // for strafing, compensate for the error
        Inches *= 1.2;

        Ticks = InchesToTicks(Inches);

        ResetEncoders(Robot);

        Robot.motorFrontLeft.setTargetPosition(-Ticks);
        Robot.motorFrontRight.setTargetPosition(Ticks);
        Robot.motorBackLeft.setTargetPosition(Ticks);
        Robot.motorBackRight.setTargetPosition(-Ticks);

        SetMotorPower(Robot, 1);
        WaitForMotors(Robot);
        SetMotorPower(Robot, 0);

        HelperSleep(SleepTime);
    }

    private void HelperSleep(long SleepTime)
    {
        try
        {
            Thread.sleep(SleepTime);
        }
        catch (InterruptedException e)
        {

        }
    }

    public void StrafeRight(RobotHWMap robot, double inches, long SleepTime)
    {
        StrafeLeft(robot, -inches, SleepTime);
    }

    public void DriveForward(RobotHWMap robot, double inches, long SleepTime)
    {
        int Ticks = InchesToTicks(inches);
        ResetEncoders(robot);

        robot.motorFrontLeft.setTargetPosition(Ticks);
        robot.motorFrontRight.setTargetPosition(Ticks);
        robot.motorBackLeft.setTargetPosition(Ticks);
        robot.motorBackRight.setTargetPosition(Ticks);

        SetMotorPower(robot, 1);
        WaitForMotors(robot);
        SetMotorPower(robot, 0);

        HelperSleep(SleepTime);
    }

    public void DriveBackward(RobotHWMap robot, double inches, long SleepTime)
    {
        DriveForward(robot, -inches, SleepTime);
    }

    public void TurnLeft(RobotHWMap robot, double inches, long SleepTime)
    {
        int Ticks = InchesToTicks(inches);
        ResetEncoders(robot);

        robot.motorFrontLeft.setTargetPosition(-Ticks);
        robot.motorFrontRight.setTargetPosition(Ticks);
        robot.motorBackLeft.setTargetPosition(-Ticks);
        robot.motorBackRight.setTargetPosition(Ticks);

        SetMotorPower(robot, 1);
        WaitForMotors(robot);
        SetMotorPower(robot, 0);
        HelperSleep(SleepTime);
    }

    public void TurnRight(RobotHWMap robot, double inches, long SleepTime)
    {
        TurnLeft(robot, inches, SleepTime);
    }

    public void KnockOffGold(HardwareMap hardwareMap, RobotHWMap robot, Telemetry telemetry)
    {
        int i = 32;
        int StrafeUnit = 4;

        while (i>=0)
        {
            // hsvValues is an array that will hold the hue, saturation, and value information.
            float hsvValues[] = {0F, 0F, 0F};
            int RGB[] = {0, 0, 0};
            double distance;

            distance = GetHSV(hardwareMap, hsvValues, RGB);
            float hue = hsvValues[0];

            PrintColorDistance(hsvValues, RGB, distance, telemetry);

            if (distance == Float.NaN)
            {
                i-=StrafeUnit;
                StrafeLeft(robot, StrafeUnit, 500);
            }
            else
            {
                if (distance > 15)
                {
                    // we are far away
                    DriveForward(robot, 1, 1000);
                }
                else
                {
                    if (hue <= 50)
                    {
                        DriveForward(robot, 6, 1000);
                        DriveBackward(robot, 6, 1000);
                        break;
                    }
                    else
                    {
                        i -= StrafeUnit;
                        StrafeLeft(robot, StrafeUnit, 500);
                    }
                }
            }
        }

        StrafeLeft(robot, (i + 10), 1000);
    }

    private double GetHSV(HardwareMap hardwareMap, float []hsvValues, int []RGB)
    {
        ColorSensor sensorColor;
        DistanceSensor sensorDistance;

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

        // change the background color to match the color detected by the RGB sensor.
        // pass a reference to the hue, saturation, and value array as an argument
        // to the HSVToColor method.
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
            }
        });

        RGB[0] = (int) (sensorColor.red() * SCALE_FACTOR);
        RGB[1] = (int) (sensorColor.green() * SCALE_FACTOR);
        RGB[2] = (int) (sensorColor.blue() * SCALE_FACTOR);

        return sensorDistance.getDistance(DistanceUnit.CM);
    }

    public void PrintColorDistance(float []hsvValues, int [] RGB, double distance, Telemetry telemetry)
    {
        // send the info back to driver station using telemetry function.
        telemetry.addData("Distance (cm)",
                String.format(Locale.US, "%.02f", distance));

        telemetry.addData("distance real", distance);

        //telemetry.addData("Alpha", sensorColor.alpha());
        telemetry.addData("Red  ", RGB[0]);
        telemetry.addData("Green", RGB[1]);
        telemetry.addData("Blue ", RGB[2]);
        telemetry.addData("Hue", hsvValues[0]);

        telemetry.update();
    }
}