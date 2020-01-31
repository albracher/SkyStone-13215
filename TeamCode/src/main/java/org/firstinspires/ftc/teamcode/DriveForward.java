package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
/*
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
*/

@Autonomous(name="Spin Stuff", group="autonomous")

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
        telemetry.addData("STATUS", "HARDWARE INITIALIZED");
        telemetry.update();

        // Resets encoder values to prevent the robot from freaking out as soon as we init
        robot.motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.motorRL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.motorRR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("STATUS", "MOTORS CONFIGURED");
        telemetry.update();


        //set servo starting positions



        //send telemetry
        telemetry.addData("STATUS", "ALL SYSTEMS GO");
        telemetry.update();

        waitForStart();

        telemetry.addData("STATUS", "ALL SYSTEMS GO");
        telemetry.update();

        //move forward to foundation
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
        telemetry.update();

    }
}