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


@Autonomous(name="Auto Mode: Blue Depot")

public class AutoBlueDepot extends LinearOpMode {

    /* Declare OpMode members. */
    RobotHWMap robot   = new RobotHWMap();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // 1440 for Tetrix motor, 1120 for our AndyMark motors
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP (ours is 1 to 1)
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.7;
    static final double     TURN_SPEED              = 0.5;

    static final double     ARM_SPEED               = 0.4;

    @Override public void runOpMode() {

        robot.init(hardwareMap);

//        // Define and initialize color sensor
//        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
//        colorSensor.enableLed(true);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // ACTUAL STUFF THAT HAPPENS

//        jewelArmDown(2.0);
//        jewel(0.70);
//        jewelArmUp(2.0);

        robot.motorBackLeft.setDirection(DcMotor.Direction.FORWARD);
        robot.motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        robot.motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        robot.motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
//        robot.motorUpDown.setDirection(DcMotor.Direction.FORWARD);
//
//        robot.motorUpDown.setPower(ARM_SPEED);
        sleep(400);

        encoderDrive(DRIVE_SPEED, 4, 4, 2.0);

        telemetry.update();

        sleep(5000);
    }


    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.motorBackLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.motorBackRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.motorFrontLeft.setTargetPosition(newLeftTarget);
            robot.motorFrontRight.setTargetPosition(newRightTarget);
            robot.motorBackLeft.setTargetPosition(newLeftTarget);
            robot.motorBackRight.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.motorFrontLeft.setPower(Math.abs(speed));
            robot.motorFrontRight.setPower(Math.abs(speed));
            robot.motorBackLeft.setPower(Math.abs(speed));
            robot.motorBackRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.motorFrontLeft.isBusy() && robot.motorFrontRight.isBusy() && robot.motorBackLeft.isBusy() && robot.motorBackRight.isBusy())) {
            }

            // Stop all motion;
            robot.motorFrontLeft.setPower(0);
            robot.motorFrontRight.setPower(0);
            robot.motorBackLeft.setPower(0);
            robot.motorBackRight.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    // jewel knocker on RED ALLIANCE
    public void jewel(double holdTime)
    {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        while (opModeIsActive() && holdTimer.seconds() < holdTime)
        {
            // mount color sensor on front of jewel arm
            if (robot.colorSensor.blue() > 0 && robot.colorSensor.blue() > robot.colorSensor.red())
            {
                encoderDrive(TURN_SPEED, 4, -4, 2.0);
                encoderDrive(TURN_SPEED, -4, 4, 2.0);
            }
            else if (robot.colorSensor.red() > 0 && robot.colorSensor.red() > robot.colorSensor.blue())
            {
                encoderDrive(TURN_SPEED, -4, 4, 2.0);
                encoderDrive(TURN_SPEED, 4, -4, 2.0);
            }
        }
    }

//    // function to set jewel arm down with enough time
//    public void jewelArmDown(double holdTime)
//    {
//        ElapsedTime holdTimer = new ElapsedTime();
//        holdTimer.reset();
//        robot.jewelServo.setPosition(0);
//        sleep(2000);
////        while (opModeIsActive() && holdTimer.seconds() < holdTime)
////        {
////
////        }
//    }

//    // function to set jewel arm down with enough time
//    public void jewelArmUp(double holdTime)
//    {
//        ElapsedTime holdTimer = new ElapsedTime();
//        holdTimer.reset();
//        robot.jewelServo.setPosition(0.65);
//        sleep(2000);
////        while (opModeIsActive() && holdTimer.seconds() < holdTime)
////        {
////
////        }
//    }
}