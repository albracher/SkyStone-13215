package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TeleOpMap {
    /* Public OpMode members. */
    public DcMotor motorFR = null;
    public DcMotor motorFL = null;
    public DcMotor motorRR = null;
    public DcMotor motorRL = null;
    public DcMotor intakeL = null;
    public DcMotor intakeR = null;
    public DcMotor armR = null;
    public DcMotor armL = null;

    public Servo claw    = null;
    public static final double MID_SERVO       =  0.5 ;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public TeleOpMap() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        motorFR = hwMap.get(DcMotor.class, "fr");
        motorFL = hwMap.get(DcMotor.class, "fl");
        motorRR = hwMap.get(DcMotor.class, "rr");
        motorRL = hwMap.get(DcMotor.class, "rl");
        intakeL = hwMap.get(DcMotor.class, "il");
        intakeR = hwMap.get(DcMotor.class, "ir");
        armL = hwMap.get(DcMotor.class, "al");
        armR = hwMap.get(DcMotor.class, "ar");
        claw  = hwMap.get(Servo.class, "c");

        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorRR.setDirection(DcMotor.Direction.FORWARD);
        motorRL.setDirection(DcMotor.Direction.REVERSE);
        intakeR.setDirection(DcMotor.Direction.FORWARD);
        intakeL.setDirection(DcMotor.Direction.REVERSE);
        armL.setDirection(DcMotor.Direction.REVERSE);
        armR.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        motorFR.setPower(0);
        motorFL.setPower(0);
        motorRR.setPower(0);
        motorRL.setPower(0);
        intakeR.setPower(0);
        intakeL.setPower(0);
        armL.setPower(0);
        armR.setPower(0);

        //set zero power behavior
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        armR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Not using encoders for non drive train to allow for more direct control of power.
        //Arm uses encoders to make sure motors stay in sync
        //same with intake
    }
}