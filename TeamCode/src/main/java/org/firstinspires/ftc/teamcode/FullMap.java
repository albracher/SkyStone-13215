package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.*;

import static java.lang.Thread.sleep;

public class FullMap {

    //TODO: Add back rotate function, try to make drive and strafe more accurate

    /* Public OpMode members. */
    public DcMotor motorFR = null;
    public DcMotor motorFL = null;
    public DcMotor motorRR = null;
    public DcMotor motorRL = null;
    public DcMotor intakeL = null;
    public DcMotor intakeR = null;
    public DcMotor slides = null;
    public Servo foundL;
    public Servo foundR;
    public DcMotor pinion;
    public Servo claw;
    public Servo autonArm;
    public Servo autonClaw;
    public Servo marker;
    public static final double MID_SERVO       =  0.5 ;
    static final int COUNTS_PER_MOTOR_REV = 2240;    // eg: Andymark Motor Encoder (40:1)
    static final double DRIVE_GEAR_REDUCTION = 15/20;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_ROTATION = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;     //used to compute degrees
    public static final double M = (2 / Math.sqrt(2));
    static final double INCHES = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION / (WHEEL_DIAMETER_INCHES * Math.PI))*M; //calculates counts per inch
    static final double FEET = 12 * INCHES;
    int OFFSET = 0;
    public static final double DRIVE_SPEED = 0.5;
    //counts per inch: 189.072002609

    private ElapsedTime runtime = new ElapsedTime();
    private Telemetry telemetry;
    private Orientation angles;

    private BNO055IMU imu;
    private BNO055IMU.Parameters gyroParameters;

    private double heading;
    private double error;
    private double lastangle = 0;
    private boolean ccwRotation = false;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public FullMap() {
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
        motorFL = hwMap.get(DcMotor.class, "fl");
        motorFR = hwMap.get(DcMotor.class, "fr");
        motorRL = hwMap.get(DcMotor.class, "rl");
        motorRR = hwMap.get(DcMotor.class, "rr");
        intakeL = hwMap.get(DcMotor.class, "il");
        intakeR = hwMap.get(DcMotor.class, "ir");
        slides = hwMap.get(DcMotor.class, "s");
        foundL = hwMap.get(Servo.class, "fol");
        foundR = hwMap.get(Servo.class, "for");
        pinion = hwMap.get(DcMotor.class, "p");
        claw = hwMap.get(Servo.class, "c");
        autonArm = hwMap.get(Servo.class, "a1");
        autonClaw = hwMap.get(Servo.class, "a2");
        marker = hwMap.get(Servo.class, "m");

        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorRR.setDirection(DcMotor.Direction.REVERSE);
        motorRL.setDirection(DcMotor.Direction.FORWARD);
        intakeR.setDirection(DcMotor.Direction.FORWARD);
        intakeL.setDirection(DcMotor.Direction.REVERSE);
        slides.setDirection(DcMotor.Direction.FORWARD);
        foundL.setDirection(Servo.Direction.REVERSE);
        foundR.setDirection(Servo.Direction.FORWARD);
        pinion.setDirection(CRServo.Direction.FORWARD);
        claw.setDirection(Servo.Direction.FORWARD);
        marker.setDirection(Servo.Direction.FORWARD);


        // Set all motors to zero power
        motorFR.setPower(0);
        motorFL.setPower(0);
        motorRR.setPower(0);
        motorRL.setPower(0);
        intakeR.setPower(0);
        intakeL.setPower(0);
        slides.setPower(0);

        //set zero power behavior
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Not using encoders for non drive train to allow for more direct control of power.
        //Arm uses encoders to make sure motors stay in sync
        //same with intake
    }

    public void slide(double speed, int distance) {

        // Resets encoder values so that it doesn't attempt to run to outdated values
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Declares target point storage variables
        int targetS;
        // Determines new target position, and pass to motor controller
        targetS = motorFL.getCurrentPosition() + distance;
        slides.setTargetPosition(targetS);

        // Sets motors to run to a given encoder value
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Motors are set to run at a certain speed until one reaches its target position
        while (slides.isBusy()) {
            slides.setPower(Math.abs(speed));
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
        }
        // The motors are shutdown when a motor gets to its target position
        slides.setPower(0);
        targetS=0;
    }


    public void drive(double speed, int distance) {
        // Resets encoder values so that it doesn't attempt to run to outdated values
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Declares target point storage variables
        int targetFL=0;
        int targetFR=0;
        int targetRL=0;
        int targetRR=0;
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

        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // The motors are shutdown when a motor gets to its target position
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorRL.setPower(0);
        motorRR.setPower(0);
        targetFL=0;
        targetFR=0;
        targetRL=0;
        targetRR=0;
        speed=0;
        distance=0;
    }

    public void strafe(double speed, int distance) {
        // Resets encoder values so that it doesn't attempt to run to outdated values
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Declares target point storage variables
        int targetFL=0;
        int targetFR=0;
        int targetRL=0;
        int targetRR=0;
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

        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // The motors are shutdown when a motor gets to its target position
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorRR.setPower(0);
        motorRL.setPower(0);

        targetFL=0;
        targetFR=0;
        targetRL=0;
        targetRR=0;
        speed=0;
        distance=0;
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



                motorFR.setPower(-speed);
                motorFL.setPower(speed);
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

                motorFR.setPower(speed);
                motorFL.setPower(-speed);
                motorRR.setPower(speed);
                motorRL.setPower(-speed);
            }
        }

        lastangle = heading;
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorRR.setPower(0);
        motorRL.setPower(0);
    }


    public void autonExtend(){

        autonArm.setPosition(0);
    }

    public void autonGrab(){
        autonClaw.setPosition(1);
        autonArm.setPosition(0);
    }

    public void autonRelease(){
        autonArm.setPosition(1);
        autonClaw.setPosition(0);
        autonClaw.setPosition(0);
    }


    public void initialApproach(int y){
        int x = 1450;
        x*=y;
        strafe(1,x);
    }

    public void approachStone(int y){
        int x = 350;
        x*=y;
        strafe(1,x);
    }

    public void bridgeAlignment(int y){
        int x = -1000;
        x*=y;
        strafe(1,x);
    }

    public void cross(int y){
        int x = 6300;
        x*=y;
        drive(1,x);
    }

    public void park(int y){
        int x = -4100;
        x*=y;
        drive(1,x);
    }

    public void foundationGrab(){
        foundL.setPosition(1);
        foundR.setPosition(1);
    }

    public void foundationRelease(){
        foundL.setPosition(0);
        foundR.setPosition(0);
    }

        public void timeRotate(double speed, double time) {
            motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorRL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorRR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            runtime.reset();
            while (runtime.seconds() < time) {
                motorFL.setPower(speed);
                motorRL.setPower(speed);
                motorFR.setPower(-speed);
                motorRR.setPower(-speed);
            }
            motorFL.setPower(0);
            motorRL.setPower(0);
            motorFR.setPower(0);
            motorRR.setPower(0);
        }

    public void strafe(double speed, int distance, double angle) {
        // Resets encoder values so that it doesn't attempt to run to outdated values
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        // Declares target point storage variables
        int targetFL=0;
        int targetFR=0;
        int targetRL=0;
        int targetRR=0;
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
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);

            heading = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
            heading = Math.floor(heading);
            heading = Range.clip(heading, -180.0, 180.0);
            error = (angle - heading)*0.1;
            motorFL.setPower(Math.abs(speed)+error);
            motorFR.setPower(Math.abs(speed)+error);
            motorRL.setPower(Math.abs(speed)-error);
            motorRR.setPower(Math.abs(speed)-error);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
        }
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorRR.setPower(0);
        motorRL.setPower(0);

        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // The motors are shutdown when a motor gets to its target position


        targetFL=0;
        targetFR=0;
        targetRL=0;
        targetRR=0;
        speed=0;
        distance=0;
    }

}