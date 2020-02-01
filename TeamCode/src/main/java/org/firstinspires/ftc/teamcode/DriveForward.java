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



        //send telemetry
        telemetry.addData("Status", "Ready to run Test Autonomous");
        telemetry.update();

        waitForStart();


        //move forward to foundation
<<<<<<< HEAD
        sleep(20000);
        robot.drive(DRIVE_SPEED, -3000);

        sleep(250);

        //clear claw for TeleOp
        robot.autonClaw.setPosition(0.7);


        telemetry.addData("Status", "I've got a good lock! Firing!");

=======
        robot.motorFL.setPower(0.5);
        robot.motorFR.setPower(0.5);
        robot.motorRL.setPower(0.5);
        robot.motorRR.setPower(0.5);
        telemetry.addData("STATUS", "ALL MOTORS SET TO 0.5 POWER");
        telemetry.update();
        sleep(10000);
        robot.motorFL.setPower(-0.5);
        robot.motorFR.setPower(-0.5);
        robot.motorRL.setPower(-0.5);
        robot.motorRR.setPower(-0.5);
        telemetry.addData("STATUS", "ALL MOTORS SET TO -0.5 POWER");
>>>>>>> parent of 0d62c37... testing front motors
        telemetry.update();

    }
}