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
    FullMap robot = new FullMap();   //Configs hardware


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

        boolean adjustedInput = false;
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
        robot.autonArm.setPosition(0.5);
        robot.autonClaw.setPosition(0.5);
        robot.foundL.setPosition(0.5);
        robot.foundR.setPosition(0.5);
        robot.marker.setPosition(0.5);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Robot is waiting.");
        telemetry.update();
        waitForStart();

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        telemetry.addData("Status", "Robot can drive.");
        telemetry.update();

        while (opModeIsActive()) {

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
                if (drive > newDrive) {
                    newDrive += 0.06;
                } else if (drive < newDrive) {
                    newDrive -= 0.06;
                }
                if (strafe > newStrafe) {
                    newStrafe += 0.06;
                } else if (strafe < newStrafe) {
                    newStrafe -= 0.06;
                }
                if (turn > newTurn) {
                    newTurn += 0.06;
                } else if (turn < newTurn) {
                    newTurn -= 0.06;
                }
            } else {
                drive = Math.pow(drive, 3);
                strafe = Math.pow(strafe, 3);
                turn = Math.pow(turn, 3);
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

            //gamepad 1 true/false controls below
            if (gamepad1.left_stick_button) {
                speed = 1;
            }
            if (gamepad1.right_stick_button) {
                speed = 0.5;
            }
            while (gamepad1.a) {
                robot.intakeL.setPower(0.9);
                robot.intakeR.setPower(0.9);
            }
            while (gamepad1.b) {
                robot.intakeL.setPower(-0.9);
                robot.intakeR.setPower(-0.9);
            }
            while (gamepad1.x) {
                robot.intakeL.setPower(0);
                robot.intakeR.setPower(0);
            }
            while (gamepad1.y) {
                robot.intakeL.setPower(0);
                robot.intakeR.setPower(0);
            }
            if (gamepad1.dpad_up) {
                robot.pinion.setPower(1);
                if (gamepad1.dpad_down) {
                    robot.pinion.setPower(-1);
                } else {
                    robot.pinion.setPower(0);
                }
            }
            while (gamepad1.dpad_left) {
            }
            while (gamepad1.dpad_right) {
            }

            if (gamepad1.left_bumper) {
                robot.claw.setPosition(0);
            }
            if (gamepad1.right_bumper) {
                robot.claw.setPosition(1);
            }

            //left and right trigger values are used to calculate armSpeed
            armSpeed = 1 * (gamepad2.right_trigger - gamepad2.left_trigger);

            //armSpeed is applied to motors
            robot.slides.setPower(armSpeed);

            // Move both servos to new position.  Assume servos are mirror image of each other.
            clawOffset = Range.clip(clawOffset, -0.5, 0.5);
            clawOffset2 = Range.clip(clawOffset2, -0.5, 0.5);
            clawOffset3 = Range.clip(clawOffset3, -0.5, 0.5);

            //player 2 controls claw with A and B


            telemetry.addData("Status", "Speed: " + speed + "\n" +
                    "Turn: " + newTurn + "        Strafe: " + newStrafe + "\n" +
                    "Slide Power: " + armSpeed + "     intake Power: " + INTAKE_SPEED + "\n");
            telemetry.update();
        }
    }
}
//idle();