package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

// Instructions

@TeleOp
public class trainingCode extends LinearOpMode {

     private DcMotor lf;
     private DcMotor lb;
     private DcMotor rf;
    private DcMotor rb;

   @Override
    public void runOpMode() throws InterruptedException {

          lf = hardwareMap.get(DcMotor.class, "lf");
         lb = hardwareMap.get(DcMotor.class, "lb");
          rf = hardwareMap.get(DcMotor.class, "rf");
          rb = hardwareMap.get(DcMotor.class, "rb");

        rf.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.REVERSE);

       /* _.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); */

        waitForStart();

        lf.setPower(gamepad1.left_stick_y);
        lb.setPower(gamepad1.left_stick_y);
        rf.setPower(gamepad1.left_stick_y);
        rb.setPower(gamepad1.left_stick_y);

        if(gamepad1.right_stick_x != 0){

            rf.setPower(-gamepad1.right_stick_x);
            rb.setPower(-gamepad1.right_stick_x);
            lf.setPower(gamepad1.right_stick_x);
            lb.setPower(gamepad1.right_stick_x);

        }
    }
}
