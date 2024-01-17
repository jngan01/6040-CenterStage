package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class armTest extends LinearOpMode {

    private DcMotor arm;
    private Servo rotator;
    private Servo wrist;
    private CRServo clamp;
    private CRServo drone;

    private int armPos;


    @Override
    public void runOpMode() throws InterruptedException {

        arm = hardwareMap.get(DcMotor.class, "arm");
        rotator = hardwareMap.get(Servo.class, "rotator");
        wrist = hardwareMap.get(Servo.class, "wrist");
        clamp = hardwareMap.get(CRServo.class, "clamp");
        drone = hardwareMap.get(CRServo.class, "drone");


        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER );

        armPos = 0;
        wrist.setPosition(0.05);
        rotator.setPosition(0);

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {

            telemetry.update();
            telemetry.addData("Arm Position:", armPos);



            if(gamepad1.dpad_left){
                rotator.setPosition(.4);
            } else if(gamepad1.dpad_up){
                rotator.setPosition(0);
            } else if(gamepad1.dpad_right){
                rotator.setPosition(.75);
            }else if(gamepad1.dpad_down){
                rotator.setPosition(.5);
            }
            if(gamepad1.x){
                drone.setPower(1);
            } else{
                drone.setPower(0);
            }
            if(gamepad1.y){
                wrist.setPosition(1);
            }
            if(gamepad1.left_bumper){
                clamp.setPower(.5);
            }
            if(gamepad1.right_bumper){
                clamp.setPower(-.5);
            }
            if(gamepad1.left_trigger > 0){

              wrist.setPosition(.4);
            }
            if(gamepad1.right_trigger > 0){

                wrist.setPosition(.6);
            }


            // Reset the arm and claw positions
            if(gamepad1.b){

                clamp.setPower(1);
                sleep(250);
                rotator.setPosition(0);
                sleep(250);
                wrist.setPosition(.05);
                sleep(250);
                //moveArm(0, .25);

                //Pick up pixel and bring up.
            } else if(gamepad1.a){

                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER );
                clamp.setPower(-1);
                sleep(500);
                moveArm(-100, .35);
                sleep(500);
                wrist.setPosition(1);

            } else if(!gamepad1.a && !gamepad1.b){

                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER );
                arm.setPower(gamepad1.left_stick_x * .5);
            }





        }

    }

    //Move arm to target position at a set speed. Once target position is reached, revert controls back to driver.
    private void moveArm(int armTarget, double speed){

        armPos += armTarget;

        arm.setTargetPosition(armPos);

        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        arm.setPower(speed);


        while(opModeIsActive() && arm.isBusy()){
            idle();
        }
    }
}
