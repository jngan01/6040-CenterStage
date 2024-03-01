package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class liftTest extends LinearOpMode{

    private DcMotor rightHang;
    private DcMotor leftHang;
    private DcMotor slides;

    @Override
    public void runOpMode() throws InterruptedException {

        rightHang = hardwareMap.get(DcMotor.class, "rightHang");
        leftHang = hardwareMap.get(DcMotor.class, "leftHang");
        slides = hardwareMap.get(DcMotor.class, "slides");

        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftHang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightHang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();


        while (!isStopRequested() && opModeIsActive()) {
            slides.setPower(gamepad2.right_stick_y * .1);

        }
    }
}
