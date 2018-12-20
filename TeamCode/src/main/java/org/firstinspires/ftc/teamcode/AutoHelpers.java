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

public class AutoHelpers {
    private MecanumHelpers Mecanum;

    AutoHelpers() {
        Mecanum = new MecanumHelpers();
    }

    private void DropTeamMarker(RobotHWMap robot) {
        //open the claw
        robot.robotClaw.setPosition(0.2);
        Mecanum.HelperSleep(500);
        robot.robotClaw.setPosition(0.8);
    }

    private void KnockGoldMineral(RobotHWMap robot)
    {
        int SleepTime = 100;

        // knock cube
        Mecanum.StrafeLeft(
                robot,
                12,
                SleepTime);

        // move away from cube
        Mecanum.StrafeRight(
                robot,
                13,
                SleepTime);

    }

    public void DescendRobot(RobotHWMap robot) {
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

        Mecanum.HelperSleep(10000);
        robot.motorLift.setPower(0);
    }

    public double GetGoldMineralPosition(HardwareMap hardwareMap) {
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
        Mecanum.HelperSleep(SleepTime);

        XPosition = detector.getXPosition();
        detector.disable();

        return XPosition;
    }


    public void Crater(RobotHWMap robot, HardwareMap hardwareMap) {
        double XPosition;
        long SleepTime = 100;
        XPosition = GetGoldMineralPosition(hardwareMap);

        // disengage the robot from the central lander
        Mecanum.DriveForward(robot, 3, SleepTime);

        int DistanceToTurn = 32; // Was 31
        int DistanceToPerimeter = 3;

        // move away from lander
        Mecanum.StrafeLeft(
                robot,
                12,
                SleepTime);

        if (XPosition < 150) {
            // Left side

            // move to cube
            Mecanum.DriveBackward(
                    robot,
                    17,
                    SleepTime);

            KnockGoldMineral(robot);

            // go to the end of the fence
            Mecanum.DriveBackward(
                    robot,
                    DistanceToPerimeter + 30,
                    SleepTime);
        }
        else if (XPosition >= 150 && XPosition <= 390)
        {
            // Center

            // move to cube
            Mecanum.DriveBackward(
                    robot,
                    5,
                    SleepTime);

            KnockGoldMineral(robot);

            // go to the end of the fence
            Mecanum.DriveBackward(
                    robot,
                    DistanceToPerimeter + 47,
                    SleepTime);
        }
        else
        {
            // move to the cube
            Mecanum.DriveForward(
                    robot,
                    11,
                    SleepTime);

            KnockGoldMineral(robot);

            // go to the end of the fence
            Mecanum.DriveBackward(
                    robot,
                    DistanceToPerimeter + 61,
                    SleepTime);
        }

        // turn so that you are aligned to the perimeter fence
        Mecanum.TurnRight(
                robot,
                DistanceToTurn,
                SleepTime);

        // move to the depot
        Mecanum.DriveForward(
                robot,
                38,
                SleepTime);

        DropTeamMarker(robot);

        // drive to the crater
        Mecanum.DriveBackward(robot, 50, SleepTime);
        Mecanum.StrafeRight(robot, 2, SleepTime);
        Mecanum.DriveBackward(robot, 22, SleepTime);
    }


    public void Depot(RobotHWMap robot, HardwareMap hardwareMap)
    {
        double XPosition;
        long SleepTime = 100;

        XPosition = GetGoldMineralPosition(hardwareMap);

        // disengage the robot from the central lander
        Mecanum.DriveForward(robot, 3, SleepTime);

        // move away from lander
        Mecanum.StrafeLeft(
                robot,
                12,
                SleepTime);

        if (XPosition < 150)
        {
            // Left side

            // move to cube
            Mecanum.DriveBackward(
                    robot,
                    17,
                    SleepTime);

            KnockGoldMineral(robot);

            // go to the end of the fence
            Mecanum.DriveBackward(
                    robot,
                    32,
                    SleepTime);
        }
        else if (XPosition >=150 && XPosition <=390)
        {
            // Center

            // move to cube
            Mecanum.DriveBackward(
                    robot,
                    5,
                    SleepTime);

            KnockGoldMineral(robot);

            // go to the end of the fence
            Mecanum.DriveBackward(
                    robot,
                    47,
                    SleepTime);

        }
        else
        {
            // move to the cube
            Mecanum.DriveForward(
                    robot,
                    11,
                    SleepTime);

            KnockGoldMineral(robot);

            // go to the end of the fence
            Mecanum.DriveBackward(
                    robot,
                    60,
                    SleepTime);


        }

        // turn so that you are aligned to the perimeter fence
        Mecanum.TurnLeft(
                robot, 14,
                SleepTime);

        // move to the depot
        Mecanum.DriveForward(
                robot,
                49,
                SleepTime);

        DropTeamMarker(robot);

        // drive to the crater
        Mecanum.DriveBackward(robot, 59, SleepTime);
    }
}