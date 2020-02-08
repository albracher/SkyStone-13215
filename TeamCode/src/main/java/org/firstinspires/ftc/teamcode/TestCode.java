package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
/*
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
*/

@Autonomous(name="Test Code", group="autonomous")

/* Declare OpMode members. */

public class TestCode extends LinearOpMode {

    FullMap robot = new FullMap();

/*
    private GoldAlignDetector detector;
*/

    private ElapsedTime runtime = new ElapsedTime();

    //Set DriveSpeed to 1 to be faster
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

        waitForStart();


        //move forward to foundation
        //sleep for 20 seconds
        sleep(2000);
        //set distance from -3000 to 2500 because st  uff flipped
        drive(DRIVE_SPEED, 2500);

        sleep(250);

    }

    public void drive(double speed, int distance) {
        // Resets encoder values so that it doesn't attempt to run to outdated values
        robot.motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Declares target point storage variables
        int targetFL=0;
        int targetFR=0;
        int targetRL=0;
        int targetRR=0;
        // Determines new target position, and pass to motor controller
        targetFL = robot.motorFL.getCurrentPosition() + distance;
        targetFR = robot.motorFR.getCurrentPosition() + distance;
        targetRL = robot.motorRL.getCurrentPosition() + distance;
        targetRR = robot.motorRR.getCurrentPosition() + distance;
        robot.motorFL.setTargetPosition(targetFL);
        robot.motorFR.setTargetPosition(targetFR);
        robot.motorRL.setTargetPosition(targetRL);
        robot.motorRR.setTargetPosition(targetRR);

        // Sets motors to run to a given encoder value
        robot.motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Motors are set to run at a certain speed until one reaches its target position
        while (robot.motorFL.isBusy() || robot.motorFR.isBusy() || robot.motorRL.isBusy() || robot.motorRR.isBusy()) {
            robot.motorFL.setPower(Math.abs(speed));
            robot.motorFR.setPower(Math.abs(speed));
            robot.motorRL.setPower(Math.abs(speed));
            robot.motorRR.setPower(Math.abs(speed));
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            telemetry.addData("Busy", " " +robot.motorFL.isBusy() +" "+robot.motorFR.isBusy() + " "+robot.motorRL.isBusy() + " "+robot.motorRR.isBusy());
            telemetry.addData("Current", " "+robot.motorFL.getCurrentPosition() + " "+robot.motorFR.getCurrentPosition() + " "+robot.motorRL.getCurrentPosition() +" "+ robot.motorRR.getCurrentPosition());
            telemetry.addData("Target", " "+targetFL + " "+targetFR + " "+targetRL +" "+ targetRR);
            telemetry.update();
        }

        robot.motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.motorRR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.motorRL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // The motors are shutdown when a motor gets to its target position
        robot.motorFL.setPower(0);
        robot.motorFR.setPower(0);
        robot.motorRL.setPower(0);
        robot.motorRR.setPower(0);
        targetFL=0;
        targetFR=0;
        targetRL=0;
        targetRR=0;
        speed=0;
        distance=0;
    }
}