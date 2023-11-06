package org.firstinspires.ftc.teamcode.autonomous;
import android.util.Size;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

import java.util.concurrent.TimeUnit;

@Autonomous
public class PowerPlayLeftAutoWithAprilTags extends LinearOpMode{

    private DcMotor lf;
    private DcMotor lb;
    private DcMotor rf;
    private DcMotor rb;
    private DcMotor fourBar;
    //private CRServo claw;

    private ElapsedTime runtime = new ElapsedTime();


    /*private int leftFrontPos;
    private int leftBackPos;
    private int rightFrontPos;
    private int rightBackPos; */

    @Override
    public void runOpMode() throws InterruptedException {

        lf = hardwareMap.get(DcMotor.class, "lf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rf = hardwareMap.get(DcMotor.class, "rf");
        rb = hardwareMap.get(DcMotor.class, "rb");
        fourBar = hardwareMap.get(DcMotor.class, "fourBar");
        //claw = hardwareMap.crservo.get("clawIntake");

        rf.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.REVERSE);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /* lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER );
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER );
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER );
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER ); */

        /*leftFrontPos = 0;
        leftBackPos = 0;
        rightFrontPos = 0;
        rightBackPos = 0; */

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();

        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {

        }

        /*BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters); */

        waitForStart();


        //claw.setPower(1);
        //sleep(2000);
        fourBar.setPower(.9);
        sleep(800);
        fourBar.setPower(0.25);
        telemetry.addLine("Pre-AprilTag Detection");

        while (!isStopRequested() && opModeIsActive()) {

            if (tagProcessor.getDetections().size() > 0) {
                org.firstinspires.ftc.vision.apriltag.AprilTagDetection tag = tagProcessor.getDetections().get(0);

                telemetry.addData("Id", tag.id);

                //telemetry.addLine(String.format("XYZ %6.2f $6.2f %6.2f", tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z));

                //A lot of these are optional. Use whatever.
                //telemetry.addData("Center", tag.center);
                //telemetry.addData("roll", tag.ftcPose.roll);
                //telemetry.addData("pitch", tag.ftcPose.pitch);
                //telemetry.addData("yaw", tag.ftcPose.yaw);

                //telemetry.addData("range", tag.ftcPose.range);
                //telemetry.addData("bearing", tag.ftcPose.bearing);
                //telemetry.addData("elevation", tag.ftcPose.elevation);

                if (tag.id == 0) {

                telemetry.addLine("Tag 1");


                    /*driveEncoder(50, 50, 50, 50, .25);
                    sleep(2000);
                    driveEncoder(-600, 600, 600, -600, .25);
                    sleep(2000);
                    driveEncoder(820, 820, 820, 820, 0.25);

                    fourBar.setPower(-.5);
                    sleep(750);
                    claw.setPower(-1);
                    sleep(750);

                    driveEncoder(-900, -900, -900, -900, .25);
                    sleep(2000);
                    driveEncoder(50, 50, 50, 50, .25);
                    sleep(2000);
                    driveEncoder(-450, 450, 450, -450, .5);
                    sleep(2000);
                    driveEncoder(800, 800, 800, 800, .75); */

                    /*while (runtime.seconds() < 1) {
                        drive(.25);
                    } */

                    /* strafe(-.5);
                    sleep(1250);
                    drive(.5);
                    sleep(600);
                    fourBar.setPower(-.5);
                    sleep(750);
                    claw.setPower(-1);
                    sleep(750);
                    drive(-.75);
                    sleep(500);
                    drive(.25);
                    sleep(250);
                    drive(0);
                    strafe(-.5);
                    sleep(1850);
                    strafe(0);
                    drive(.5);
                    sleep(1250);
                    drive(0);
                    stop(); */

                    drive(.25);
                    sleep(250);
                    drive(0);
                    strafe(-.5);
                    sleep(2500);
                    strafe(0);
                    drive(.5);
                    sleep(1250);
                    drive(0);
                    fourBar.setPower(0.0);
                    stop();

                } else if(tag.id == 1){

                    telemetry.addLine("Tag 2");

                    /*driveEncoder(50, 50, 50, 50, .25);
                    driveEncoder(-600, 600, 600, -600, .25);
                    sleep(500);
                    driveEncoder(820, 820, 820, 820, 0.25);

                    fourBar.setPower(-.5);
                    sleep(750);
                    claw.setPower(-1);
                    sleep(750);

                    driveEncoder(-900, -900, -900, -900, .25);
                    driveEncoder(50, 50, 50, 50, .25);
                    driveEncoder(-450, 450, 450, -450, .5);
                    driveEncoder(800, 800, 800, 800, .75);*/

                    drive(.25);
                    sleep(250);
                    drive(0);
                    strafe(-.5);
                    sleep(300);
                    strafe(0);
                    drive(.5);
                    sleep(1250);
                    drive(0);
                    fourBar.setPower(0.0);
                    stop();


                } else if(tag.id == 2){

                    telemetry.addLine("Tag 3");
/*
                    driveEncoder(50, 50, 50, 50, .25);
                    sleep(2000);
                    driveEncoder(-600, 600, 600, -600, .25);
                    sleep(2000);
                    driveEncoder(820, 820, 820, 820, 0.25);

                    fourBar.setPower(-.5);
                    sleep(750);
                    claw.setPower(-1);
                    sleep(750);

                    driveEncoder(-900, -900, -900, -900, .25);
                    sleep(2000);
                    driveEncoder(50, 50, 50, 50, .25);
                    sleep(2000);
                    driveEncoder(-450, 450, 450, -450, .5);
                    sleep(2000);
                    driveEncoder(800, 800, 800, 800, .75); */

                    drive(.25);
                    sleep(250);
                    drive(0);
                    strafe(.5);
                    sleep(2500);
                    strafe(0);
                    drive(.5);
                    sleep(1200);
                    drive(0);
                    fourBar.setPower(0.0);
                    stop();

                }

            }
        }


        while (opModeIsActive() && lf.isBusy() && lb.isBusy() && rf.isBusy() && rb.isBusy()) {
            idle();
        }
    }
    public void strafe(double direction) {

        lf.setPower(1 * direction);
        lb.setPower(-1 * direction);
        rf.setPower(-1 * direction);
        rb.setPower(1 * direction);

    }
    public void drive(double direction){
        lf.setPower(direction);
        lb.setPower(direction);
        rf.setPower(direction);
        rb.setPower(direction);
    }

    /*private void driveEncoder(int lfTarget, int lbTarget, int rfTarget, int rbTarget, double speed)
    {
        leftFrontPos += lfTarget;
        leftBackPos += lbTarget;
        rightFrontPos += rfTarget;
        rightBackPos += rbTarget;

        lf.setTargetPosition(leftFrontPos);
        lb.setTargetPosition(leftBackPos);
        rf.setTargetPosition(rightFrontPos);
        rb.setTargetPosition(rightBackPos);

        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lf.setPower(speed);
        lb.setPower(speed);
        rf.setPower(speed);
        rb.setPower(speed);
    } */


}

