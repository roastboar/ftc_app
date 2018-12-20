package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumHelpers
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

    public void HelperSleep(long SleepTime)
    {
        try
        {
            Thread.sleep(SleepTime);
        }
        catch (InterruptedException e)
        {

        }
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

    public void StrafeLeft(RobotHWMap robot, double Inches, long SleepTime)
    {
        int Ticks;

        // for strafing, compensate for the error
        Inches *= 1.2;

        Ticks = InchesToTicks(Inches);

        robot.ResetEncoders(robot);
        robot.RunToPosition(robot);

        robot.motorFrontLeft.setTargetPosition(-Ticks);
        robot.motorFrontRight.setTargetPosition(Ticks);
        robot.motorBackLeft.setTargetPosition(Ticks);
        robot.motorBackRight.setTargetPosition(-Ticks);

        SetMotorPower(robot, 1);
        WaitForMotors(robot);
        SetMotorPower(robot, 0);

        HelperSleep(SleepTime);
    }


    public void StrafeRight(RobotHWMap robot, double inches, long SleepTime)
    {
        StrafeLeft(robot, -inches, SleepTime);
    }

    public void DriveForward(RobotHWMap robot, double inches, long SleepTime)
    {
        int Ticks = InchesToTicks(inches);
        robot.ResetEncoders(robot);
        robot.RunToPosition(robot);

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
        robot.ResetEncoders(robot);
        robot.RunToPosition(robot);

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
}