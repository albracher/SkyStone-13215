package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
/*
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
*/

@Autonomous(name="Blue Park", group="autonomous")

/* Declare OpMode members. */

public class Bluepark extends LinearOpMode {

    ExperimentalAutonMap robot = new ExperimentalAutonMap();

/*
    private GoldAlignDetector detector;
*/

    private ElapsedTime runtime = new ElapsedTime();

    public static double DRIVE_SPEED = 0.5;

    @Override

    public void runOpMode() {
        robot.init(hardwareMap);

        // Resets encoder values to prevent the robot from freaking out as soon as we init
        robot.motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //send telemetry
        telemetry.addData("Status", "Ready to run Test Autonomous");
        telemetry.update();

        robot.foundL.setPosition(0.2);
        robot.foundR.setPosition(0.2);
//
        waitForStart();
//        //move forward to foundation
//        robot.drive(1, -2100);
//        robot.strafe(1,-800);
//        //attach the arm to the foundation
//        robot.foundL.setPosition(1);
//        robot.foundR.setPosition(1);
//        sleep(2000);
//        //pull back foundation
//
//        robot.drive(1, 1500);


        robot.drive(1,-3000);

        //let go
//        robot.foundL.setPosition(0);
//        robot.foundR.setPosition(0);
//        sleep(1000);
//        robot.drive(1,3550);
//        //move under bridge



        telemetry.addData("Status", "Auton complete.");


        telemetry.update();

    }
}