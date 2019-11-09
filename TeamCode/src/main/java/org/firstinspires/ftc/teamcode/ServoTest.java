package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
/*
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
*/

@Autonomous(name="Servo 0.5", group="F.A.R.T.")

/* Declare OpMode members. */


public class ServoTest extends LinearOpMode {

    AutonMap robot = new AutonMap();

    private ElapsedTime runtime = new ElapsedTime();

    public static double DRIVE_SPEED = 0.5;

    /*
    tile size is 24 inches
    660 counts of encoder = 4 inches
    1 inch = 165 counts
    */

    @Override

    public void runOpMode() {
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

        robot.autonClaw.setPosition(0);            // S4: Stop and close the claw.
        robot.autonClamp.setPosition(0);

        waitForStart();
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
//            // Play the audio
//            //SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, soundID);
//            robot.rotate("ccw", 0.4, 170);
            robot.autonClaw.setPosition(0.5);            // S4: Stop and close the claw.
            robot.autonClamp.setPosition(0.5);


            telemetry.addData("Status", "I've got a good lock! Firing!");
            telemetry.update();

            //ONE TILE IS 24 INCHES X 24 INCHES

            //drive to crater


            //test intake
/*        intake(0.5, 3);
        sleep(1000);
        //test actuator
        actuate(1, 5);*/
        }
}