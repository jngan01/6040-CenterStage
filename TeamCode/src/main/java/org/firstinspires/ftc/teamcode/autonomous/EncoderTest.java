package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class EncoderTest extends LinearOpMode{

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    private int leftFrontPos;
    private int leftBackPos;
    private int rightFrontPos;
    private int rightBackPos;


    private DcMotor fourBar;
    private DcMotor topLift;
    private DcMotor bottomLift;

    private CRServo claw;

    @Override
    public void runOpMode() throws InterruptedException{

        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");

        fourBar = hardwareMap.dcMotor.get("fourBar");
        topLift = hardwareMap.dcMotor.get("liftTop");
        bottomLift = hardwareMap.dcMotor.get("liftBottom");

        claw = hardwareMap.crservo.get("clawIntake");

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fourBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Encoder Part
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER );
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER );
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER );
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER );

        leftFrontPos = 0;
        leftBackPos = 0;
        rightFrontPos = 0;
        rightBackPos = 0;

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        waitForStart();

        claw.setPower(1);
        sleep(1000);
        fourBar.setPower(.7);
        sleep(800);
        fourBar.setPower(0.25);

        drive(-450, 450, 450, -450, .25);
        sleep(500);
        drive(820, 820, 820, 820, 0.25);

        fourBar.setPower(-.5);
        sleep(750);
        claw.setPower(-1);
        sleep(750);

        drive(-900, -900, -900, -900, .25);
        drive(-600, 600, 600, -600, .25);

    }

    private void drive(int lfTarget, int lbTarget, int rfTarget, int rbTarget, double speed){
        leftFrontPos += lfTarget;
        leftBackPos += lbTarget;
        rightFrontPos += rfTarget;
        rightBackPos += rbTarget;

        leftFront.setTargetPosition(leftFrontPos);
        leftBack.setTargetPosition(leftBackPos);
        rightFront.setTargetPosition(rightFrontPos);
        rightBack.setTargetPosition(rightBackPos);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(speed);
        leftBack.setPower(speed);
        rightFront.setPower(speed);
        rightBack.setPower(speed);

        while(opModeIsActive() && leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()){
            idle();
        }
    }

}
