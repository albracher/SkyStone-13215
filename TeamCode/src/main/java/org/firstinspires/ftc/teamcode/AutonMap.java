package org.firstinspires.ftc.teamcode;

import android.graphics.Point;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
/*import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;*/
import java.util.*;

public class AutonMap {
    /* Public OpMode members. */
    public DcMotor motorFR = null;
    public DcMotor motorFL = null;
    public DcMotor motorRR = null;
    public DcMotor motorRL = null;
    private double x = 0;
    private double y = 0;

   /* private GoldAlignDetector detector;*/
    private ElapsedTime runtime = new ElapsedTime();
    private Telemetry telemetry;
    private Orientation angles;

    private BNO055IMU imu;
    private BNO055IMU.Parameters gyroParameters;

    private double heading;
    private double lastangle = 0;
    private boolean ccwRotation = false;

 static final int COUNTS_PER_MOTOR_REV = 2240;    // eg: Andymark Motor Encoder (40:1)
    static final double DRIVE_GEAR_REDUCTION = 15/20;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_ROTATION = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;     //used to compute degrees
    public static final double M = (2 / Math.sqrt(2));
    static final double INCHES = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION / (WHEEL_DIAMETER_INCHES * Math.PI))*M; //calculates counts per inch
    static final double FEET = 12 * INCHES;
    //59.41785 base inches (no mec compensation)
    //84.02952 per inch
    int OFFSET = 0;
    public static final double DRIVE_SPEED = 0.5;

    public static final double MID_SERVO = 0.5;//legacy code, can be removed
    //public ColorSensor colorSensor;//legacy code, can be removed

    //YellowJacket for Close
    //NeverRest for Open

    //NeverRest 40 motor: Diameter = 0.85 inches
    //NeverREST: 120 rpm
    //Circumference = 2.669 inches
    //NeverRest: IPM: 320.28 inches/min
    //GoBilda Yellow Jacket motor: Diameter = 1 inch
    //GoBilda: 84 rpm
    //Circumference: 3.14 inches
    //GoBilda: IPM: 263.76 inches/min
    //YellowJacket is close
    //NeverRest is open
    //Run NeverRest at 82.35% to get same power

    /* local OpMode members. */

    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();
    /* Constructor */
    public AutonMap() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Define and Initialize Motors
        motorFR = hwMap.get(DcMotor.class, "fr");
        motorFL = hwMap.get(DcMotor.class, "fl");
        motorRR = hwMap.get(DcMotor.class, "rr");
        motorRL = hwMap.get(DcMotor.class, "rl");

        motorFR.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if motors are facing outward
        motorFL.setDirection(DcMotor.Direction.REVERSE); // Set to FORWARD if motors are facing outward
        motorRR.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if motors are facing outward
        motorRL.setDirection(DcMotor.Direction.REVERSE); // Set to FORWARD if motors are facing outward

        // Sets zero power behavior to brake for more precise movement
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Resets encoder values to prevent the robot from freaking out as soon as we init
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set all motors to zero power
        motorFR.setPower(0);
        motorFL.setPower(0);
        motorRR.setPower(0);
        motorRL.setPower(0);

        // fucking gyro init hell yeet dab
    }

    //use these for calculating changes in x and y for odometry
    //they did the monster math
    //it was a graveyard graph
    public void updateX(int dist){

    }
    public void updateY(int dist){
    }




    public void drive(double speed, int distance) {
        // Resets encoder values so that it doesn't attempt to run to outdated values
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Declares target point storage variables
        int targetFL;
        int targetFR;
        int targetRL;
        int targetRR;
        // Determines new target position, and pass to motor controller
        targetFL = motorFL.getCurrentPosition() + distance;
        targetFR = motorFR.getCurrentPosition() + distance;
        targetRL = motorRL.getCurrentPosition() + distance;
        targetRR = motorRR.getCurrentPosition() + distance;
        motorFL.setTargetPosition(targetFL);
        motorFR.setTargetPosition(targetFR);
        motorRL.setTargetPosition(targetRL);
        motorRR.setTargetPosition(targetRR);

        // Sets motors to run to a given encoder value
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Motors are set to run at a certain speed until one reaches its target position
        while (motorFL.isBusy() && motorFR.isBusy() && motorRL.isBusy() && motorRR.isBusy()) {
            motorFL.setPower(Math.abs(speed));
            motorFR.setPower(Math.abs(speed));
            motorRL.setPower(Math.abs(speed));
            motorRR.setPower(Math.abs(speed));
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
        }
        // The motors are shutdown when a motor gets to its target position
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorRL.setPower(0);
        motorRR.setPower(0);
        targetFL=0;
        targetFR=0;
        targetRL=0;
        targetRR=0;
    }

    public void rotate(String direction, double speed, double angle) {

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);

        heading = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
        heading = Math.floor(heading);
        heading = Range.clip(heading, -180.0, 180.0);

        //boolean beforeAngle = (direction.equals("cw") ? heading > angle | heading<angle);

        if (direction.equals("cw") || direction.equals("clockwise")) {
            while (Math.abs(heading) > angle) {

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                heading = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);

                motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorRL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorRR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



                motorFR.setPower(speed);
                motorFL.setPower(-speed);
                motorRR.setPower(-speed);
                motorRL.setPower(speed);

            }
        } else if (direction.equals("counterclockwise") || direction.equals("ccw")) {
            while (heading < angle) {

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                heading = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);

                motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorRL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorRR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                motorFR.setPower(-speed);
                motorFL.setPower(speed);
                motorRR.setPower(speed);
                motorRL.setPower(-speed);
            }
        }

        lastangle = heading;
    }

    public void strafe(double speed, int distance) {
        // Resets encoder values so that it doesn't attempt to run to outdated values
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Declares target point storage variables
        int targetFL;
        int targetFR;
        int targetRL;
        int targetRR;
        // Determines new target position, and pass to motor controller
        targetFL = motorFL.getCurrentPosition() + distance;
        targetFR = motorFR.getCurrentPosition() - distance;
        targetRL = motorRL.getCurrentPosition() - distance;
        targetRR = motorRR.getCurrentPosition() + distance;
        motorFL.setTargetPosition(targetFL);
        motorFR.setTargetPosition(targetFR);
        motorRL.setTargetPosition(targetRL);
        motorRR.setTargetPosition(targetRR);

        // Sets motors to run to a given encoder value
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Motors are set to run at a certain speed until one reaches its target position
        while (motorFL.isBusy() && motorFR.isBusy() && motorRL.isBusy() && motorRR.isBusy()) {
            motorFL.setPower(Math.abs(speed));
            motorFR.setPower(Math.abs(speed));
            motorRL.setPower(Math.abs(speed));
            motorRR.setPower(Math.abs(speed));
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
        }
        // The motors are shutdown when a motor gets to its target position
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorRR.setPower(0);
        motorRL.setPower(0);

        targetFL=0;
        targetFR=0;
        targetRL=0;
        targetRR=0;
    }


































    /*    public void actuate(double speed, double time) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData("Status:", "Actuating", runtime.seconds());
            telemetry.update();
            robot.actuator.setPower(speed);
        }
    }*/

    /*public void intake(double speed, double time) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData("Status:", "Actuating", runtime.seconds());
            telemetry.update();
            robot.intake.setPower(speed);
        }
    }*/
}