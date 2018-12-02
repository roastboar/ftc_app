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


@Autonomous(name="Auto Facing Depot")

public class AutoFacingDepot extends LinearOpMode
{
    /* Declare OpMode members. */
    RobotHWMap robot = new RobotHWMap();
    AutoHelpers AutoHelper = new AutoHelpers();

    @Override
    public void runOpMode()
    {
        robot.init(hardwareMap);

        // close the claw servo
        //robot.robotClaw.setPosition(0.8);

        AutoHelper.ResetEncoders(robot);

        waitForStart();

        // close the claw servo
        //robot.robotClaw.setPosition(0.8);

        double XPosition;
        XPosition = AutoHelper.GetGoldMineralPosition(hardwareMap, 3000);

        // disengage the robot from the central lander
        AutoHelper.DriveForward(robot, 3, 1000);

        int DistanceToCubeTotal = 29;
        int DistanceToCubeInitial = 12;
        int DistanceToStrafe = 18;
        int DistanceToBackoff = 19;

        // move away from lander
        AutoHelper.StrafeLeft(
                robot,
                12,
                1000);

        if (XPosition < 150)
        {
            // Left side

            // move to cube
            AutoHelper.DriveBackward(
                    robot,
                    17,
                    1000);

            // knock cube
            AutoHelper.StrafeLeft(
                    robot,
                    12,
                    1000);

            // move away from cube
            AutoHelper.StrafeRight(
                    robot,
                    13,
                    1000);

            // go to the end of the fence
            AutoHelper.DriveBackward(
                    robot,
                    32,
                    1000);

        }
        else if (XPosition >=150 && XPosition <=390)
        {
            // Center

            // move to cube
            AutoHelper.DriveBackward(
                    robot,
                    5,
                    1000);

            // knock off cube
            AutoHelper.StrafeLeft(
                    robot,
                    12,
                    1000);

            // move away from cube
            AutoHelper.StrafeRight(
                    robot,
                    13,
                    1000);

            // go to the end of the fence
            AutoHelper.DriveBackward(
                    robot,
                    47,
                    1000);
        }
        else
        {
            // move to the cube
            AutoHelper.DriveForward(
                    robot,
                    11,
                    1000);

            // knock off the cube
            AutoHelper.StrafeLeft(
                    robot,
                    12,
                    1000);

            // move away from cube
            AutoHelper.StrafeRight(
                    robot,
                    13,
                    1000);

            // go to the end of the fence
            AutoHelper.DriveBackward(
                    robot,
                    60,
                    1000);
        }

        // turn so that you are aligned to the perimeter fence
        AutoHelper.TurnLeft(
                robot, 14,
                1000);

        // move to the depot
        AutoHelper.DriveForward(
                robot,
                49,
                1000);

        //open the claw
        robot.robotClaw.setPosition(0);

        // drive to the crater
        AutoHelper.DriveBackward(robot, 57, 1000);
    }
}