/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


@Autonomous(name="Basic Auto")

public class BasicAuto extends LinearOpMode
{
    /* Declare OpMode members. */
    RobotHWMap robot = new RobotHWMap();

    @Override
    public void runOpMode()
    {
        robot.init(hardwareMap);
        robot.colorServo.setPosition(0.60);

        ResetEncoders();

        waitForStart();

        // descend the robot
        // to be done later to avoid burning the rack and pinion gear

        // disengage the robot from the central lander
        NewStrafeRight(4);
        sleep(1000);

        // drive forward till we reach the minerals
        NewDriveForward(17.5);
        sleep(1000);

        // strafe to get to the first mineral
        NewStrafeRight(11);
        sleep(1000);

        boolean GoldMineralSeen = false;
        for (int i=0; i<2;i++)
        {
            if (GoldMineralSeen == false && !IsWhiteMineral()) {
                GoldMineralSeen = true;
                NewDriveForward(6);
                sleep(1000);
                NewDriveBackward(6);
            }
            sleep(1000);
            NewStrafeLeft(12);
        }

        robot.colorServo.setPosition(0.93);
    }

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

    public int InchesToTicks(double inches)
    {
        // Our wheels are 4"
        // One rotation will result in a travel of 12.56 inches.
        // (circumference=diameter*pi)

        // Neverest has 1120 ticks per rotation
        return (int) (inches/(12.56/1120));
    }

    public void ResetEncoders()
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

    public void WaitForMotors()
    {
        while(
                (robot.motorFrontLeft.isBusy() &&
                 robot.motorFrontRight.isBusy() &&
                 robot.motorBackLeft.isBusy() &&
                 robot.motorBackRight.isBusy()
                ) && opModeIsActive())
        {
            // nothing
        }
    }

    public void SetMotorPower(int value)
    {
        robot.motorFrontLeft.setPower(value);
        robot.motorFrontRight.setPower(value);
        robot.motorBackLeft.setPower(value);
        robot.motorBackRight.setPower(value);
    }

    public void NewStrafeLeft(double inches)
    {
        int Ticks;

        // for strafing, compensate for the error
        inches *= 1.2;

        Ticks = InchesToTicks(inches);

        ResetEncoders();

        robot.motorFrontLeft.setTargetPosition(-Ticks);
        robot.motorFrontRight.setTargetPosition(Ticks);
        robot.motorBackLeft.setTargetPosition(Ticks);
        robot.motorBackRight.setTargetPosition(-Ticks);

        SetMotorPower(1);
        WaitForMotors();
        SetMotorPower(0);

        return;
    }

    public void NewStrafeRight(double inches)
    {
        NewStrafeLeft(-inches);
    }

    public void NewDriveForward(double inches)
    {
        int Ticks = InchesToTicks(inches);
        ResetEncoders();

        robot.motorFrontLeft.setTargetPosition(Ticks);
        robot.motorFrontRight.setTargetPosition(Ticks);
        robot.motorBackLeft.setTargetPosition(Ticks);
        robot.motorBackRight.setTargetPosition(Ticks);

        SetMotorPower(1);
        WaitForMotors();
        SetMotorPower(0);
    }

    public void NewDriveBackward(double inches)
    {
        NewDriveForward(-inches);
    }

    public void NewTurnLeft(double inches)
    {
        int Ticks = InchesToTicks(inches);
        ResetEncoders();

        robot.motorFrontLeft.setTargetPosition(-Ticks);
        robot.motorFrontRight.setTargetPosition(Ticks);
        robot.motorBackLeft.setTargetPosition(-Ticks);
        robot.motorBackRight.setTargetPosition(Ticks);

        SetMotorPower(1);
        WaitForMotors();
        SetMotorPower(0);
    }

    public void NewTurnRight(double inches)
    {
        NewTurnLeft(inches);
    }

    public boolean IsWhiteMineral()
    {
        /*
        int threshold = 60;
        if (robot.colorSensor.blue() > threshold &&
                robot.colorSensor.green() > threshold &&
                robot.colorSensor.blue() > threshold)
        {
            return true;
        }*/
        int alphaThreshold = 175;
        if (robot.colorSensor.alpha() > alphaThreshold)
        {
            return true;
        }
        return false;
    }

    public boolean IsGoldMineral()
    {
        if (robot.colorSensor.blue() < 30 && robot.colorSensor.blue() > 15)
        {
            return true;
        }
        return false;
    }
}

