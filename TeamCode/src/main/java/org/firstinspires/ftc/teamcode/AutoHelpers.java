package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;

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

    public void DescendRobot(RobotHWMap robot)
    {
        int Ticks;
        double Inches = 6.5;

        robot.motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

/*
        robot.motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Ticks = InchesToTicks(Inches);
        robot.motorLift.setTargetPosition(Ticks);
        SetMotorPower(robot, 1);
        WaitForMotors(robot);
        SetMotorPower(robot, 0);
        HelperSleep(SleepTime);
*/
        //
        // this was used for rack and pinion gear
        //
        robot.motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorLift.setPower(-1);

        // 4475 -- 4750

        HelperSleep(15000);
        robot.motorLift.setPower(0);
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
        TurnLeft(robot, -inches, SleepTime);
    }

    public double GetGoldMineralPosition(HardwareMap hardwareMap)
    {
        GoldAlignDetector detector;
        double XPosition;
        long SleepTime = 2000;

        // telemetry.addData("Status", "DogeCV 2018.0 - Gold Align Example");

        // Set up detector
        detector = new GoldAlignDetector(); // Create detector

        // Initialize it with the app context and camera
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());

        detector.useDefaults(); // Set detector to use default settings

        // Optional tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005; //

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment

        detector.enable(); // Start the detector!

        // Sleep to allow it to calibrate
        HelperSleep(SleepTime);

        XPosition = detector.getXPosition();
        detector.disable();

        return XPosition;
    }
}