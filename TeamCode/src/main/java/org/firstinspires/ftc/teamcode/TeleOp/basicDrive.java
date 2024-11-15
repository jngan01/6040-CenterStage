package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Disabled

public class basicDrive extends LinearOpMode {

   private DcMotor lf;
   private DcMotor lb;
   private DcMotor rf;
   private DcMotor rb;

    private DcMotor rightHang;
    private DcMotor arm;
    private Servo rotator;
    private Servo wrist;
    private Servo clamp;
    private CRServo drone;
    private Servo leftIntake;
    private Servo rightIntake;
    private DcMotor leftSlides;
    private DcMotor rightSlides;


    private DistanceSensor distanceSensorLeft;
    private DistanceSensor distanceSensorRight;
    private int armPos;

    @Override
    public void runOpMode() throws InterruptedException {

        lf = hardwareMap.get(DcMotor.class, "lf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rf = hardwareMap.get(DcMotor.class, "rf");
        rb = hardwareMap.get(DcMotor.class, "rb");

        rightHang = hardwareMap.get(DcMotor.class, "rightHang");
        arm = hardwareMap.get(DcMotor.class, "arm");
        rotator = hardwareMap.get(Servo.class, "rotator");
        wrist = hardwareMap.get(Servo.class, "wrist");
        clamp = hardwareMap.get(Servo.class, "clamp");
        drone = hardwareMap.get(CRServo.class, "drone");
        leftIntake = hardwareMap.get(Servo.class, "leftIntake");
        rightIntake = hardwareMap.get(Servo.class, "rightIntake");
        distanceSensorLeft = hardwareMap.get(DistanceSensor.class, "distanceSensorLeft");
        distanceSensorRight = hardwareMap.get(DistanceSensor.class, "distanceSensorRight");
        leftSlides = hardwareMap.get(DcMotor.class, "leftSlides");
        rightSlides = hardwareMap.get(DcMotor.class, "rightSlides");


        rightSlides.setDirection(DcMotor.Direction.REVERSE);

        boolean pixelScannerOn = false;



        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER );

        armPos = 0;
        wrist.setPosition(0);
        rotator.setPosition(0);
        boolean isOpen = false;


        rf.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.REVERSE);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        waitForStart();


        while (!isStopRequested() && opModeIsActive()) {

            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x; //* 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            arm.setPower(gamepad2.left_stick_y * .7);

            /*if(gamepad2.left_stick_y > 0){
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER );
                arm.setPower(gamepad2.left_stick_y * .7);
            } else{
                //arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER );
                armPos = arm.getCurrentPosition();
                arm.setTargetPosition(armPos);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            } */


            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            telemetry.update();

            rightHang.setPower(gamepad2.right_stick_x);

            leftSlides.setPower(gamepad2.right_stick_y);
            leftSlides.setPower(gamepad2.right_stick_y);

            //Distance Sensor controls
            if(gamepad1.a){

                leftIntake.setPosition(1);
                rightIntake.setPosition(0);

            }

            if(gamepad1.b){
                pixelScannerOn = true;
            }

            if(pixelScannerOn == true){
                if(distanceSensorLeft.getDistance(DistanceUnit.CM) <= 5.2 || distanceSensorRight.getDistance(DistanceUnit.CM) <= 5.2){

                    leftIntake.setPosition(0);
                    rightIntake.setPosition(1);
                    pixelScannerOn = false;
                    telemetry.update();
                }
            }
            if(gamepad1.y){
                pixelScannerOn = false;
                leftIntake.setPosition(0);
                rightIntake.setPosition(1);
            }



            if (gamepad1.right_bumper) {
                lf.setPower(-.4 * frontLeftPower);
                lb.setPower(-.4 * backLeftPower);
                rf.setPower(-.4 * frontRightPower);
                rb.setPower(-.4 * backRightPower);
            } else {
                lf.setPower(-1 * frontLeftPower);
                lb.setPower(-1 * backLeftPower);
                rf.setPower(-1 * frontRightPower);
                rb.setPower(-1 * backRightPower);
            }

            // Orient pixels
            if(gamepad2.dpad_left){
                rotator.setPosition(.4);
            } else if(gamepad2.dpad_up){
                rotator.setPosition(0);
            } else if(gamepad2.dpad_right){
                rotator.setPosition(.75);
            }else if(gamepad2.dpad_down){
                rotator.setPosition(.5);
            }

            // Launch Drone
            if(gamepad1.x){
                drone.setPower(1);
            } else{
                drone.setPower(0);
            }

            //Intake
            if(gamepad2.y){

                leftIntake.setPosition(1);
                rightIntake.setPosition(0);

            } else if(gamepad2.x) {
                leftIntake.setPosition(0);
                rightIntake.setPosition(1);

            }

            // Pixel release failsafe
            if(gamepad2.left_bumper){
                clamp.setPosition(0);
            }
            if(gamepad2.right_bumper){
                clamp.setPosition(1);
            }

            // To adjust claw angle when placing on the Backdrop
            if(gamepad2.left_trigger > 0){

                wrist.setPosition(.4);
            }
            if(gamepad2.right_trigger > 0){

                wrist.setPosition(.6);
            }


            // Reset the arm and claw positions
           if(gamepad2.b){

                clamp.setPosition(1);
                rotator.setPosition(0);
                sleep(250);
                wrist.setPosition(.0);

                //Pick up pixel and bring up.
            } /* else if(gamepad2.a){

                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER );
                clamp.setPower(-1);
                sleep(500);
                moveArm(-100, .35);
                sleep(500);
                wrist.setPosition(1);

                //When not in autonomous mode, the arm can be controlled manually for more specific heights.
            } else if(!gamepad2.a && !gamepad2.b){

                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER );
                arm.setPower(gamepad2.left_stick_x * .5);
            } */
        }

    }
    //Move arm to target position at a set speed. Once target position is reached, revert controls back to driver.
    private void moveArm(int armTarget, double speed) {

        armPos += armTarget;

        arm.setTargetPosition(armPos);

        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        arm.setPower(speed);


        while (opModeIsActive() && arm.isBusy()) {
            idle();
        }


    }
}
