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
import com.qualcomm.robotcore.hardware.DistanceSensor;
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
    AutoHelpers AutoHelper = new AutoHelpers();

    @Override
    public void runOpMode()
    {
        robot.init(hardwareMap);

        robot.colorServo.setPosition(0);

        //robot.colorServo.setPosition(0.60);

        AutoHelper.ResetEncoders(robot);

        waitForStart();

        // close the claw servo
        robot.robotClaw.setPosition(0.8);

        // set initial position for the color servo
        robot.colorServo.setPosition(0.33);

        // descend the robot
        AutoHelper.DescendRobot(robot);

        // disengage the robot from the central lander
        AutoHelper.StrafeRight(robot, 4, 1000);

        // drive forward till we reach the minerals
        AutoHelper.DriveForward(robot, 16,1000);

        // set the color servo to scan position
        //robot.colorServo.setPosition(0);

        // strafe to get to the first mineral
        //AutoHelper.StrafeRight(robot, 11, 1000);

        //AutoHelper.KnockOffGold(hardwareMap, robot, telemetry);

        // get to the end of the perimeter fence
        AutoHelper.StrafeLeft(robot, 35, 1000);

        // turn so that you are aligned to the perimeter fence
        AutoHelper.TurnRight(robot, 5, 1000);

        // drive to the depot
        AutoHelper.DriveForward(robot, 50, 1000);

        // open the claw
        robot.robotClaw.setPosition(0);

        // drive to the crater
        AutoHelper.DriveBackward(robot, 76, 1000);
    }
}