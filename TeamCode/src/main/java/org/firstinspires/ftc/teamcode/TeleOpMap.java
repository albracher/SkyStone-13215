package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TeleOpMap {
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
    //copied stuff from AutonMap


    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();
    private Telemetry telemetry;

    /* Constructor */
    public TeleOpMap() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

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
}