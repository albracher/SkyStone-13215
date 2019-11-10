package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
/*
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
*/

@Autonomous(name="Drive Forward 6 inches", group="autonomous")

/* Declare OpMode members. */

public class DriveForward extends LinearOpMode {

    AutonMap robot = new AutonMap();

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

        //set servo starting positions

        robot.autonClaw.setPosition(0.55);

        robot.autonClamp.setPosition(0.9);

        //send telemetry
        telemetry.addData("Status", "Ready to run Test Autonomous");
        telemetry.update();

        waitForStart();


        //move forward to foundation
        robot.drive(DRIVE_SPEED, 500);

        sleep(250);

        //clear claw for TeleOp
        robot.autonClaw.setPosition(0.7);


        telemetry.addData("Status", "I've got a good lock! Firing!");

        telemetry.update();

    }
}