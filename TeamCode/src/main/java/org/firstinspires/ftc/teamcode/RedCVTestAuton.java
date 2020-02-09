package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;


/**
 * nerverest ticks
 * 60 1680
 * 40 1120
 * 20 560
 *
 * monitor: 640 x 480
 *YES
 */
@Autonomous(name= "Red CV Side", group="CV")
public class RedCVTestAuton extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    FullMap robot = new FullMap();

    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;

    private static float rectHeight = .6f/8f;
    private static float rectWidth = 1.5f/8f;

    private static float offsetX = 0f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {4f/8f+offsetX, 7f/16f+offsetY};//0 = col, 1 = row
    private static float[] leftPos = {2f/8f+offsetX, 7f/16f+offsetY};
    private static float[] rightPos = {6f/8f+offsetX, 7f/16f+offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;

    private final double DRIVE_SPEED = 0.5;

    OpenCvCamera phoneCam;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        int position = 0;

        telemetry.addData("STATUS", "INITIALIZED");
        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.



        //"foundL and foundR" is for the foundation
        robot.foundL.setPosition(0.5);
        robot.foundR.setPosition(0.5);

        while(!opModeIsActive()){
            if (valLeft == 0) {
                position = 3;
            }
            if (valMid == 0) {
                position = 2;
            }
            if (valRight == 0) {
                position = 1;
            }

            telemetry.addData("Values", valLeft+"   "+valMid+"   "+valRight);
            telemetry.addData("Height", rows);
            telemetry.addData("Width", cols);
            telemetry.addData("Position", position);

            telemetry.update();

            sleep(1000);



        }

        waitForStart();
        runtime.reset();
        if(opModeIsActive()) {




            phoneCam.closeCameraDevice();
            robot.strafe(0.8,2575); //robot alignment is rotated, so "strafe" really just means drive towards blocks

            telemetry.addData("STATUS", "APPROACH COMPLETED"); // robot has driven up to the blocks
            telemetry.update();
             // we have 5 seconds to read telemetry before the robot decides what it wants to do

            //robot.timeRotate(-0.25, 1);

            if (position == 2) {
                //movement not required
                //robot.drive(DRIVE_SPEED, 300);
            } else if (position == 1) {
                robot.drive(DRIVE_SPEED, 800);
            } else {
                robot.drive(DRIVE_SPEED, -600);
            }

            telemetry.addData("STATUS", "CORRECTION COMPLETED");
            telemetry.update();

            robot.autonClaw.setPosition(0);
            sleep(1000);


            robot.autonArm.setPosition(0.3);
            sleep(500);



            robot.autonClaw.setPosition(0.7);

            sleep(1000);

            robot.autonArm.setPosition(0.05);

            sleep(1000);

            telemetry.addData("STATUS", "GRAB COMPLETED");
            telemetry.update();

            robot.strafe(DRIVE_SPEED, -1200, 0);

            telemetry.addData("STATUS", "REVERSE COMPLETED");
            telemetry.update();

            sleep(1000);
            robot.rotate(0.4,0);
            if(position==1) {
                robot.drive(1, -6200);
            }
            else if(position == 2){
                robot.drive(1, -5500);
            }
            else{
                robot.drive(1, -5200);
            }
            telemetry.addData("STATUS", "CROSSING COMPLETED");
            telemetry.update();
            robot.rotate(0.4,0);

            robot.strafe(0.8, 1200);


            telemetry.addData("STATUS", "RETURN COMPLETED");
            telemetry.update();

            sleep(1000);

            //drops the block
            robot.autonArm.setPosition(0.5);
            robot.autonClaw.setPosition(0);
            sleep(1000);
            robot.autonArm.setPosition(0);

            //moves to far side of foundation
            robot.drive(1,-1400);


            //align with foundation
            robot.strafe(1, 200);
            robot.autonArm.setPosition(0.05);
            //lowers foundation hooks
            robot.foundL.setPosition(0.95);
            robot.foundR.setPosition(0.95);

            robot.rotate(0.4,0);
            robot.drive(0.7,-700);

            robot.drive(0.7,400);
            //robot.rotate("cw",0.7,90);
            robot.strafe(0.9,-4600);
            robot.foundL.setPosition(0.2);
            robot.foundR.setPosition(0.2);
            robot.rotate(0.4,0);
            //drive out from under the foundation
            robot.drive(0.9, 2350);
            //align with center of bridge
            robot.strafe(0.7,2500);
            //move under bridge
            robot.drive(0.9, 1500);

        }
    }

    //detection pipeline
    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage
        {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private Stage stageToRenderToViewport = Stage.detection;
        private Stage[] stages = Stage.values();

        @Override
        public void onViewportTapped()
        {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input)
        {
            contoursList.clear();
            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */

            //color diff cb.
            //higher cb = more blue = skystone -> white
            //lower cb = less blue = yellow stone -> grey

            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame
            double[] pixMid = thresholdMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
            valMid = (int)pixMid[0];

            double[] pixLeft = thresholdMat.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
            valLeft = (int)pixLeft[0];

            double[] pixRight = thresholdMat.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
            valRight = (int)pixRight[0];

            //create three points
            Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
            Point pointLeft = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows()* leftPos[1]));
            Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointLeft,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointRight,5, new Scalar( 255, 0, 0 ),1 );//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols()*(leftPos[0]-rectWidth/2),
                            input.rows()*(leftPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(leftPos[0]+rectWidth/2),
                            input.rows()*(leftPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols()*(midPos[0]-rectWidth/2),
                            input.rows()*(midPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(midPos[0]+rectWidth/2),
                            input.rows()*(midPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols()*(rightPos[0]-rectWidth/2),
                            input.rows()*(rightPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(rightPos[0]+rectWidth/2),
                            input.rows()*(rightPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);

            switch (stageToRenderToViewport)
            {
                case THRESHOLD:
                {
                    return thresholdMat;
                }

                case detection:
                {
                    return all;
                }

                default:
                {
                    return input;
                }
            }
        }

    }
}