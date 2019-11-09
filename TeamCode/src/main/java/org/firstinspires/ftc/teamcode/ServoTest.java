package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
/*
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
*/

@TeleOp(name="Servo 0.5", group="F.A.R.T.")

/* Declare OpMode members. */


public class ServoTest extends LinearOpMode {

    AutonMap robot = new AutonMap();

    private ElapsedTime runtime = new ElapsedTime();

    public static double DRIVE_SPEED = 0.5;

    final double CLAW_SPEED = 0.001;
    private double clawOffset1 = 0;
    private double clawOffset2 = 0;
    private double clawOffset3 = 0;
    /*
    tile size is 24 inches
    660 counts of encoder = 4 inches
    1 inch = 165 counts
    */

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        /**=
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/

        // Look for the audio file
        //boolean soundFound;
        //int soundID = hardwareMap.appContext.getResources().getIdentifier("attackontitan", "raw", hardwareMap.appContext.getPackageName());

        // Preload the audio if the file has a valid ID
        //if (soundID != 0)
        //soundFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, soundID);


        //send telemetry
        telemetry.addData("Status", "Ready to run Test Autonomous");
        telemetry.update();
        robot.claw.setPosition(0.5);
        robot.autonClaw.setPosition(0.5);            // S4: Stop and close the claw.
        robot.autonClamp.setPosition(0.5);

        waitForStart();

        while (opModeIsActive()) {
//        robot.drive(0.5, -1200);
//
//        if (tfod != null) {
//            // getUpdatedRecognitions() will return null if no new information is available since
//            // the last time that call was made.
//            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
//            sleep(500);
//            if (updatedRecognitions != null) {
//
//
//                // step through the list of recognitions and display boundary info.
//                int i = 0;
//                for (Recognition recognition : updatedRecognitions) {
//                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
//                    telemetry.update()
//;                    sleep(5000);
//                    x = recognition.getLabel();
//                    if(x.equals("Stone")){
//                        robot.strafe(0.5,500);
//                        if(x.equals("Stone")) {
//                            robot.strafe(0.5, 500);
//                        }
//                    }
//
//                }
//                telemetry.update();
//            }
//
//
            if (gamepad1.right_bumper)
                clawOffset1 += CLAW_SPEED;
            else if (gamepad1.left_bumper)
                clawOffset1 -= CLAW_SPEED;

            // Move both servos to new position.  Assume servos are mirror image of each other.
            clawOffset1 = Range.clip(clawOffset1, -0.5, 0.5);

            robot.claw.setPosition(robot.MID_SERVO + clawOffset1);

            if (gamepad1.a)
                clawOffset2 += CLAW_SPEED;
            else if (gamepad1.x)
                clawOffset2 -= CLAW_SPEED;

            // Move both servos to new position.  Assume servos are mirror image of each other.
            clawOffset2 = Range.clip(clawOffset2, -0.5, 0.5);

            robot.autonClaw.setPosition(robot.MID_SERVO + clawOffset2);
            if (gamepad1.b)
                clawOffset3 += CLAW_SPEED;
            else if (gamepad1.y)
                clawOffset3 -= CLAW_SPEED;

            // Move both servos to new position.  Assume servos are mirror image of each other.
            clawOffset3 = Range.clip(clawOffset3, -0.5, 0.5);

            robot.autonClamp.setPosition(robot.MID_SERVO + clawOffset3);


            telemetry.addData("Status", "I've got a good lock! Firing!");
            telemetry.update();
        }

            //ONE TILE IS 24 INCHES X 24 INCHES

            //drive to crater


            //test intake
/*        intake(0.5, 3);
        sleep(1000);
        //test actuator
        actuate(1, 5);*/
        }
}