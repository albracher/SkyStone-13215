package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

//*In theory* this should also be compatible with tank drive, except for the strafing parts

@TeleOp(name = "Super Highfalutin TeleOp", group = "sht")
public class ExperimentalTeleOp extends LinearOpMode {
    BNO055IMU imu;



    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    public static final double ARM_SPEED = 0.925;
    public static final double INTAKE_SPEED = 0.825;

    /* Declare OpMode members. */
    TeleOpMap robot = new TeleOpMap();   //Configs hardware


    //NeverRest 40 motor: Diameter = 0.85 inches
    //NeverREST: 120 rpm
    //Circumference = 2.669 inches
    //NeverRest: IPM: 320.28 inches/min
    //GoBilda Yellow Jacket motor: Diameter = 1 inch
    //GoBilda: 84 rpm
    //Circumference: 3.14 inches
    //GoBilda: IPM: 263.76 inches/min

    //To run at the same rate, the NeverRest must run at 82.35% of it's speed
    //This puts both motors running @ approx. 263.75 in/min

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        //loads hardwareMap

        boolean adjustedInput = true;
        double drive;
        double turn;
        double strafe;
        double newDrive = 0;
        double newTurn = 0;
        double newStrafe = 0;
        double leftValue;
        double rightValue;
        double powerFL;
        double powerFR;
        double powerRL;
        double powerRR;
        double intakePower;
        double clawOffset = 0.00;
        double clawOffset2 = 0.00;
        double clawOffset3 = 0.00;
        final double CLAW_SPEED = 0.001;
        double armSpeed;
        double speed = 1;
        int counterUpTighten = 0;
        int counterUpLoosen = 0;
        int counterDownTighten = 0;
        int counterDownLoosen = 0;

        //init servos
        robot.pinion.setPower(0);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Robot is waiting.");
        telemetry.update();

        robot.foundL.setPosition(0.2);
        robot.foundR.setPosition(0.2);

        waitForStart();

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        telemetry.addData("Status", "Robot can drive.");
        telemetry.update();

        while (opModeIsActive()) {
            //speed is
            //LS is fast, RS is half speed
            if (gamepad1.left_stick_button) {
                adjustedInput = false;
                speed = 1;
            }
            if (gamepad1.right_stick_button) {
                adjustedInput = true;
                speed = 1;
            }



            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            drive = -gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;
            strafe = -gamepad1.left_stick_x;

            /*adjusted input here is meant to make robot movement less jerky
            it basically works by forcing the robot to change speed at a certain rate
            this basically means no sudden accelerations or sudden stops
             */

            if (adjustedInput) {
                if(drive>newDrive){
                    newDrive += 0.06;
                } else if (drive<newDrive){
                    newDrive -= 0.06;
                }
                if(strafe>newStrafe){
                    newStrafe += 0.06;
                } else if (strafe<newStrafe){
                    newStrafe -= 0.06;
                }
                if(turn>newTurn){
                    newTurn += 0.06;
                } else if (turn<newTurn){
                    newTurn -= 0.06;
                }
            } else {
                newDrive = drive;
                newStrafe = strafe;
                newTurn = turn;
            }

            // Combine drive and turn for blended motion.
            leftValue = newDrive - newTurn;
            rightValue = newDrive + newTurn;
            powerFL = leftValue - newStrafe;
            powerFR = rightValue + newStrafe;
            powerRL = leftValue + newStrafe;
            powerRR = rightValue - newStrafe;

            //applies acceleration curve
            //nevermind screw this, replacing with input processing
            /*powerFL *= Math.abs(powerFL);
            powerFR *= Math.abs(powerFR);
            powerRL *= Math.abs(powerRL);
            powerRR *= Math.abs(powerRR);*/

            //applies speed limiter
            powerFL *= speed;
            powerFR *= speed;
            powerRL *= speed;
            powerRR *= speed;

            //makes sure motor values aren't insane
            powerFL = Range.clip(powerFL, -speed, speed);
            powerFR = Range.clip(powerFR, -speed, speed);
            powerRL = Range.clip(powerRL, -speed, speed);
            powerRR = Range.clip(powerRR, -speed, speed);

            //sets motor power
            robot.motorFL.setPower(powerFL);
            robot.motorFR.setPower(powerFR);
            robot.motorRL.setPower(powerRL);
            robot.motorRR.setPower(powerRR);

            //rear right motor test while buttons are held
            while (gamepad1.a){
                robot.intakeL.setPower(0.9);
                robot.intakeR.setPower(0.9);
            }
            while(gamepad1.b) {
                robot.intakeL.setPower(-0.9);
                robot.intakeR.setPower(-0.9);
            }
            while(gamepad1.x){
                robot.intakeL.setPower(0);
                robot.intakeR.setPower(0);
            }

            //left and right trigger values are used to calculate armSpeed
            armSpeed= 1 *(gamepad2.right_trigger-gamepad2.left_trigger);

            //armSpeed is applied to motors
            robot.slides.setPower(armSpeed);

            if (gamepad1.dpad_up && !robot.slides.isBusy()){
                robot.slide(1, 1000);
            }
            if (gamepad1.dpad_down && !robot.slides.isBusy()){
                robot.slide(1, -1000);

            }

            //player 2 controls the pinion using right stick
            robot.pinion.setPower(gamepad2.right_stick_y);

            // Move both servos to new position.  Assume servos are mirror image of each other.
            clawOffset = Range.clip(clawOffset, -0.5, 0.5);
            clawOffset2 = Range.clip(clawOffset2, -0.5, 0.5);
            clawOffset3 = Range.clip(clawOffset3, -0.5, 0.5);

            //player 2 controls claw with A and B
            if(gamepad2.a||gamepad1.a){
                robot.claw.setPosition(0);
                robot.intakeSR.setPosition(0);
            }
            if(gamepad2.b||gamepad1.b){
                robot.claw.setPosition(1);
                robot.intakeSR.setPosition(1);
            }
            if(gamepad2.x){

            }
            if(gamepad2.y){

            }
            if(gamepad2.dpad_up){
                robot.marker.setPosition(0);
            }
            if(gamepad2.dpad_down){
                robot.marker.setPosition(1);
            }
            if(gamepad1.x){
                robot.foundL.setPosition(0);
                robot.foundR.setPosition(0);
            }
            if(gamepad1.y){
                robot.foundL.setPosition(1);
                robot.foundR.setPosition(1);
            }
            telemetry.addData("Status", "Speed: " + speed + "\n" +
                    "Power: " + drive + "        Turn: " + turn + "        Strafe: " + strafe + "\n" +
                    "Slide Power: " + "     intake Power: " + INTAKE_SPEED + "\n" +
                    "Counter Up Tighten: " + counterUpTighten + "Counter Down Tighten: " + counterDownTighten + "\n"
             + "Counter Up Loosen: " + counterUpLoosen + "Counter Down Loosen: " + counterDownLoosen);
            telemetry.update();
        }
    }
}
    //idle();
