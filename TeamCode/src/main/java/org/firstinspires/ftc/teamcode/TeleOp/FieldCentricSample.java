package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp
public class FieldCentricSample extends LinearOpMode {
    DigitalChannel digitalTouch;
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        //TouchSensor touch = hardwareMap.get(TouchSensor.class, "Touch");
        digitalTouch = hardwareMap.get(DigitalChannel.class, "digital_touch");
        DcMotor lf = hardwareMap.dcMotor.get("lf");
        DcMotor lb = hardwareMap.dcMotor.get("lb");
        DcMotor rf = hardwareMap.dcMotor.get("rf");
        DcMotor rb = hardwareMap.dcMotor.get("rb");

        DcMotor intake = hardwareMap.dcMotor.get("intake");
        DcMotor leftLift = hardwareMap.dcMotor.get("leftLift");
        DcMotor rightLift = hardwareMap.dcMotor.get("rightLift");
        DcMotor hang = hardwareMap.dcMotor.get("hang");
        CRServo drone = hardwareMap.get(CRServo.class, "drone");
        CRServo hangArm = hardwareMap.get(CRServo.class, "hangArm");

        // The holding and release of pixels
        CRServo clawArm = hardwareMap.get(CRServo.class, "clawArm");
        Servo lClamp = hardwareMap.get(Servo.class, "lClamp");
        Servo rClamp = hardwareMap.get(Servo.class, "rClamp");
        Servo rotator = hardwareMap.get(Servo.class, "clawRotator");
        Servo ClawAdjuster = hardwareMap.get(Servo.class, "rightClawAdjuster");


        digitalTouch.setMode(DigitalChannel.Mode.INPUT);
        //Servo middleGate = hardwareMap.get(Servo.class, "middleGate");


        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rightLift.setDirection(DcMotor.Direction.REVERSE);
        rClamp.setDirection(Servo.Direction.REVERSE);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Retrieve the IMU from the hardware map
        BHI260IMU imu = hardwareMap.get(BHI260IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        BHI260IMU.Parameters parameters = new BHI260IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        //ModernRoboticsI2cRangeSensor rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");

        rotator.setPosition(0);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            telemetry.update();

            /*if(digitalTouch.getState() == true){
                telemetry.addData("Digital Touch", "is NOT touched");
                scuffIntake.setPower(.1);
            }else{
                telemetry.addData("Digital Touch", "is touched");
                scuffIntake.setPower(-.5);
                sleep(2000);
            }
            telemetry.update(); */

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }


            // Driver 2 Controls

            //Rotator to position on backdrop
            if(gamepad2.dpad_up){
                rotator.setPosition(.5);
            } else if(gamepad2.dpad_left){
                rotator.setPosition(0);
            }else if(gamepad2.dpad_right){
                rotator.setPosition(1);
            }
            //Clamp for pixels
            if(gamepad2.left_bumper){
                lClamp.setPosition(0);
                rClamp.setPosition(0);
            } else if(gamepad2.right_bumper){
                lClamp.setPosition(1);
                rClamp.setPosition(1);
            }

            //Intake
            intake.setPower(gamepad2.left_trigger);
            intake.setPower(-gamepad2.right_trigger);

            //Lift
            leftLift.setPower(gamepad2.right_stick_y);
            rightLift.setPower(gamepad2.right_stick_y);

            // Claw arm
            clawArm.setPower(gamepad2.left_stick_y);
            //Hang
            if(gamepad2.a){
                hang.setPower(1);
            } else if(gamepad2.b){
                hang.setPower(-1);
            }

            //Hang extension
            if(gamepad2.x){
                hangArm.setPower(.5);
            } else if(gamepad2.y){
                hangArm.setPower(-.5);
            }

            //Drone
            if(gamepad1.x){
                drone.setPower(-.25);
            }





            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            lf.setPower(frontLeftPower);
            lb.setPower(backLeftPower);
            rf.setPower(frontRightPower);
            rb.setPower(backRightPower);
        }
    }
}