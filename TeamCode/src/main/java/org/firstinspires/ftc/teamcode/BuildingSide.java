package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
/*
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
*/

@Autonomous(name="Red Building Side", group="autonomous")

/* Declare OpMode members. */

public class BuildingSide extends LinearOpMode {

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


        //move forward to foundation
        robot.drive(1, -2200);
        robot.strafe(1,-1600);
        //attach the arm to the foundation
        robot.foundL.setPosition(1);
        robot.foundR.setPosition(1);
        robot.strafe(1,-200);
        sleep(2000);
        //pull back foundation

        robot.drive(1, 2300);


        //let go
        robot.foundL.setPosition(0);
        robot.foundR.setPosition(0);
        sleep(1000);
        robot.strafe(1,5000);
        //move under bridge



        telemetry.addData("Status", "Auton complete.");


        telemetry.update();

    }
}