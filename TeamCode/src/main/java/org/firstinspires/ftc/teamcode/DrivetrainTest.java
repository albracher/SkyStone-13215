package org.firstinspires.ftc.teamcode;

//Dpad Up = speed 100%
//Dpad Down = speed 50%
//RT Raises intake
//LT Lowers intake
//Second player can also control intake with left stick for fine control, or RT/LT
//Button A does intake (toggle)
//Button X is reverse intake (toggle)
//Button Y is slow intake (50% speed) (toggle)
//Dpad left is actuator down (hold)
//Dpad right is actuator up (hold)

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

//*In theory* this should also be compatible with tank drive, except for the strafing parts

@TeleOp(name = "TeleOp", group = "TeleOp")
public class DrivetrainTest extends LinearOpMode {
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

        double drive;
        double turn;
        double strafe;
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
        double speed = 0.5;
        int counterUpTighten = 0;
        int counterUpLoosen = 0;
        int counterDownTighten = 0;
        int counterDownLoosen = 0;

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Robot is waiting.");
        telemetry.update();

        waitForStart();

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        telemetry.addData("Status", "Robot can drive.");
        telemetry.update();

        while (opModeIsActive()) {
            //speed is
            //LS is fast, RS is half speed
            if (gamepad1.left_stick_button) {
                speed = 1;
            }
            if (gamepad1.right_stick_button) {
                speed = 0.5;
            }
            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            drive = -gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;
            //makes turn slower than everything else
            turn*= 0.5;
            strafe = gamepad1.left_stick_x;

            // Combine drive and turn for blended motion.
            leftValue = drive - turn;
            rightValue = drive + turn;
            powerFL = leftValue - strafe;
            powerFR = rightValue + strafe;
            powerRL = leftValue + strafe;
            powerRR = rightValue - strafe;

            //applies acceleration curve
            powerFL *= Math.abs(powerFL);
            powerFR *= Math.abs(powerFR);
            powerRL *= Math.abs(powerRL);
            powerRR *= Math.abs(powerRR);

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
                robot.intakeL.setPower(0.5);
                robot.intakeR.setPower(0.5);
            }
            while(gamepad1.b) {
                robot.intakeL.setPower(-0.5);
                robot.intakeR.setPower(-0.5);
            }
            while(gamepad1.x){
                robot.intakeL.setPower(0);
                robot.intakeR.setPower(0);
            }

            //left and right trigger values are used to calculate armSpeed
            armSpeed=0.45*(gamepad1.right_trigger-gamepad1.left_trigger);

            //armSpeed is applied to motors
            robot.armL.setPower(armSpeed);
            robot.armR.setPower(armSpeed);

            //Use LB and RB to open and close the claw
            if (gamepad1.right_bumper)
                clawOffset += CLAW_SPEED;
            else if (gamepad1 .left_bumper)
                clawOffset -= CLAW_SPEED;

            if (gamepad2.a)
                clawOffset2 += CLAW_SPEED;
            else if (gamepad2.y)
                clawOffset2 -= CLAW_SPEED;

            if (gamepad2.x)
                clawOffset3 += CLAW_SPEED;
            else if (gamepad2.b)
                clawOffset3 -= CLAW_SPEED;

            // Move both servos to new position.  Assume servos are mirror image of each other.
            clawOffset = Range.clip(clawOffset, -0.5, 0.5);
            clawOffset2 = Range.clip(clawOffset, -0.5, 0.5);
            clawOffset3 = Range.clip(clawOffset, -0.5, 0.5);


            robot.claw.setPosition(robot.MID_SERVO + clawOffset);
            robot.claw.setPosition(robot.MID_SERVO + clawOffset);
            robot.claw.setPosition(robot.MID_SERVO + clawOffset);


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
