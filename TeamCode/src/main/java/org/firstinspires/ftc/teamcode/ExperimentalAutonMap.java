package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

/*import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;*/

public class ExperimentalAutonMap {

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
    public CRServo pinion;
    public Servo claw;
    public Servo intakeSR;
    public Servo marker;
    private double x = 0;
    private double y = 0;

    /* private GoldAlignDetector detector;*/
    private ElapsedTime runtime = new ElapsedTime();
    private Telemetry telemetry;
    private Orientation angles;

    private BNO055IMU imu;
    private BNO055IMU.Parameters gyroParameters;

    public double robotXPos;
    public double robotYpos;
    public double heading;
    private double lastangle = 0;
    private boolean ccwRotation = false;

    static final int COUNTS_PER_MOTOR_REV = 2240;    // eg: Andymark Motor Encoder (40:1)
    static final double DRIVE_GEAR_REDUCTION = 15/20;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_ROTATION = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;     //used to compute degrees
    static final double M = (2 / Math.sqrt(2));
    static final double INCHES = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION / (WHEEL_DIAMETER_INCHES * Math.PI))*M; //calculates counts per inch
    static final double FEET = 12 * INCHES;
    //59.41785 base inches (no mec compensation)
    //84.02952 per inch
    int OFFSET = 0;
    public static final double DRIVE_SPEED = 0.5;

    public static final double MID_SERVO = 0.5;//legacy code, can be removed

    /* local OpMode members. */

    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();
    /* Constructor */
    public ExperimentalAutonMap() {

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
        pinion = hwMap.get(CRServo.class, "p");
        claw = hwMap.get(Servo.class, "c");
        intakeSR = hwMap.get(Servo.class, "isr");
        marker = hwMap.get(Servo.class, "m");

        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorRR.setDirection(DcMotor.Direction.REVERSE);
        motorRL.setDirection(DcMotor.Direction.FORWARD);
        intakeR.setDirection(DcMotor.Direction.FORWARD);
        intakeL.setDirection(DcMotor.Direction.REVERSE);
        slides.setDirection(DcMotor.Direction.FORWARD);
        foundL.setDirection(Servo.Direction.FORWARD);
        foundR.setDirection(Servo.Direction.REVERSE);
        pinion.setDirection(CRServo.Direction.FORWARD);
        claw.setDirection(Servo.Direction.FORWARD);
        intakeSR.setDirection(Servo.Direction.FORWARD);
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
        // May want to use RUN_USING_ENCODER if encoders are installed.
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Not using encoders for non drive train to allow for more direct control of power.
        //Arm uses encoders to make sure motors stay in sync
        //same with intake


    }

    //use these for calculating changes in x and y for odometry
    //they did the monster math
    //it was a graveyard graph
    // fucking gyro init hell yeet dab
    public void updateX(int dist){

    }
    public void updateY(int dist){
    }

    public void xMovement(double speed){
        motorFL.setPower(speed);
        motorFR.setPower(speed);
        motorRR.setPower(speed);
        motorRL.setPower(speed);
    }

    public void yMovement(double speed){
        motorFL.setPower(speed);
        motorFR.setPower(speed);
        motorRR.setPower(speed);
        motorRL.setPower(speed);
    }


    public void drive(double speed, int distance) {
        // Resets encoder values so that it doesn't attempt to run to outdated values
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

       /* //get start heading and init variables
        double startHeading = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
        double currentHeading;
        double difference;*/

        // Declares target point storage variables
        int targetFL;
        int initialFL;
        double completion;
        int targetFR;
        int targetRL;
        int targetRR;
        // Determines new target position, and pass to motor controller
        targetFL = motorFL.getCurrentPosition() + distance;
        //grab this for movement smoothing
        initialFL = motorFL.getCurrentPosition();
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

            //let's do fancy math to smooth movement!
            //first, we need to figure out how close we are to getting to our target
            //i'm going to be using the front left motor to calculate this
            completion = (motorFL.getCurrentPosition()-initialFL)/(targetFL-initialFL);
            completion = (-4*Math.pow((completion-0.5),2)+1.25);
            completion = Range.clip(completion, 0, 1);
            completion *= speed;

            motorFL.setPower(Math.abs(completion));
            motorFR.setPower(Math.abs(completion));
            motorRL.setPower(Math.abs(completion));
            motorRR.setPower(Math.abs(completion));
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues


            //get current heading
           /* currentHeading = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
            difference = startHeading - currentHeading;*/

            //rotate(0.5, difference);
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

    public void rotate(double speed, double angle) {

        //this is a modified version of the other angle code
        //instead of checking for clockwise or counterclockwise it checks +-
        //+ is counterclockwise i think
        //angle are absoluted before the loop that turns the bot runs because i didn't want to
        //mess with that code

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);

        heading = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
        heading = Math.floor(heading);
        heading = Range.clip(heading, -180.0, 180.0);

        //boolean beforeAngle = (direction.equals("cw") ? heading > angle | heading<angle);

        if (angle<0) {
            angle = Math.abs(angle);
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
        } else if (angle>0) {
            angle = Math.abs(angle);
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

    public void strafe(double speed, int distance) {
        // Resets encoder values so that it doesn't attempt to run to outdated values
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //get start heading and init variables
       /* double startHeading = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
        double currentHeading;
        double difference;*/

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

            //get current heading
           /* currentHeading = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
            difference = startHeading - currentHeading;

            rotate(0.5, difference);*/
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

}