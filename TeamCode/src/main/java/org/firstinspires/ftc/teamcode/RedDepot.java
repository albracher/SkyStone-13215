package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;
/*
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
*/

@Autonomous(name="Red Depot", group="autonomous")

/* Declare OpMode members. */


public class RedDepot extends LinearOpMode {

    AutonMap robot = new AutonMap();

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AWURDUL/////AAAAGRV3tgkIxUKJg8ACr6QTilZlZ1VVATMWISVYxdiTMTcFnwP0Dp8Y8L8zBqWPzaq3E6+V/lOF9bpkuQlzovfoK4UvwqKBAJMoYhVOO2hy9eiWG86YiGqEht3AyASbYaWMPLU/ckM21is0kw/GuUPtU5i6Xer7/wjdlcctLXl5I+iDFjA5NJR/eCGRmLPF5GvE73PTisGqrJLqLHXseIehtHdkieLZsRviD3uEnTQEekSHDT1VHFYvYNlFHR1V9RLWCwwe0Pf3Kdx3helXjacUUK27SMBH1RrQOA+FhT/5EfO1PFFDsrdaRGLadzY4GuJ8c5yDbrkcg56P1//tkzuDetKgYShwCM70TJd+LGYr4hUF";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;
/*
    private GoldAlignDetector detector;
*/

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
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        String x;

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfodParameters.minimumConfidence = 0.8;
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }

        // Look for the audio file
        boolean soundFound;
        //int soundID = hardwareMap.appContext.getResources().getIdentifier("attackontitan", "raw", hardwareMap.appContext.getPackageName());

        // Preload the audio if the file has a valid ID
        //if (soundID != 0)
        //soundFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, soundID);


        //send telemetry
        telemetry.addData("Status", "Ready to run Test Autonomous");
        telemetry.update();


        waitForStart();
<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/RedDepot.java

        //drive towards scanning position
        robot.drive(0.5, -1350);

        //extend claw
        robot.autonClaw.setPosition(0.55);

        //prep clamp
        robot.autonClamp.setPosition(0.95);
        sleep(500);

        telemetry.addData("vision sees: ",tfod.getUpdatedRecognitions());

        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            sleep(1000);

            if (updatedRecognitions != null) {


                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.update()
                    ;
                    sleep(1000);
                    x = recognition.getLabel();
                    if (x.equals("Stone")) {
                        robot.strafe(0.5, 500);
                        y = recognition.getLabel();
                        if (y.equals("Stone")) {
                            robot.strafe(0.5, 500);
                        }
                    }

                }
                telemetry.update();
            }


            // Play the audio
            //SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, soundID);

            //rotate to grab
            robot.rotate("ccw", 0.2, 171);
            //approach stone
            robot.drive(0.2, 350);
            //grab
=======
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
//                    ;
//                    sleep(5000);
//                    x = recognition.getLabel();
//                    if (x.equals("Stone")) {
//                        robot.strafe(0.5, 500);
//                        if (x.equals("Stone")) {
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
//            robot.autonClaw.setPosition(0.5);
            robot.rotate("ccw", 0.4, 175);
            robot.autonClamp.setPosition(0.1);
            sleep(2000);
            robot.autonClaw.setPosition(0.7);// S4: Stop and close the claw.
            sleep(2000);
>>>>>>> parent of 09afa5f... kjghkjh:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/FullAutonomousRewriteTest.java
            robot.autonClamp.setPosition(0.5);
            sleep(2000);
            robot.autonClaw.setPosition(0.2);


            telemetry.addData("Status", "I've got a good lock! Firing!");
            tfod.shutdown();
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
<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/RedDepot.java
}
=======
>>>>>>> parent of 09afa5f... kjghkjh:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/FullAutonomousRewriteTest.java