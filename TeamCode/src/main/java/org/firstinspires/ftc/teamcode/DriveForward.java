package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
/*
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
*/

@Autonomous(name="Drive Forward", group="autonomous")

/* Declare OpMode members. */

public class DriveForward extends LinearOpMode {

    FullMap robot = new FullMap();

/*
    private GoldAlignDetector detector;
*/

    private ElapsedTime runtime = new ElapsedTime();

    public static double DRIVE_SPEED = 1;

    @Override

    public void runOpMode() {
        robot.init(hardwareMap);

        // Resets encoder values to prevent the robot from freaking out as soon as we init
        robot.motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //set servo starting positions



        //send telemetry
        telemetry.addData("Status", "Ready to run Test Autonomous");
        telemetry.update();

        robot.foundL.setPosition(0.5);
        robot.foundR.setPosition(0.5);

        waitForStart();


        //move forward to foundation
        sleep(2000);
        robot.strafe(DRIVE_SPEED, 2500);

        sleep(250);

        //clear claw for TeleOp

        robot.foundL.setPosition(0.95);
        robot.foundR.setPosition(0.95);
        sleep(2000);

        robot.strafe(DRIVE_SPEED, -2500);

    }
}