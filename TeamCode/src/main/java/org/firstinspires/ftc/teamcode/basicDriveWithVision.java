package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;
import android.provider.ContactsContract;
import android.text.TextPaint;
import android.util.Size;

import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

import java.util.concurrent.TimeUnit;

@TeleOp
public class basicDriveWithVision extends LinearOpMode {

    private DcMotor lf = null;
    private DcMotor lb = null;
    private DcMotor rf = null;
    private DcMotor rb = null;

    public static final double ticksPerMotorRev = 383.6;
    public static final double driveGearReduction = 0.5;
    public static final double wheelDiameterInches = 4;
    public static final double ticksPerDriveInch = (ticksPerMotorRev * driveGearReduction) / (wheelDiameterInches * 3.14159265359);

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException{
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        lf = hardwareMap.get(DcMotor.class, "lf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rf = hardwareMap.get(DcMotor.class, "rf");
        rb = hardwareMap.get(DcMotor.class, "rb");

        rf.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.REVERSE);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

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

        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING){

        }
        ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);
        exposure.setMode(ExposureControl.Mode.Manual);
        exposure.setExposure(15, TimeUnit.MILLISECONDS);

        GainControl gain = visionPortal.getCameraControl((GainControl.class));
        gain.setGain(100);
        waitForStart();


        while (!isStopRequested() && opModeIsActive()) {

            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            if (gamepad1.right_bumper) {
                lf.setPower(.3*frontLeftPower);
                lb.setPower(.3*backLeftPower);
                rf.setPower(.3*frontRightPower);
                rb.setPower(.3*backRightPower);
            } else {
                lf.setPower(.75*frontLeftPower);
                lb.setPower(.75*backLeftPower);
                rf.setPower(.75*frontRightPower);
                rb.setPower(.75*backRightPower);
            }
            float fixAngle = gamepad1.right_trigger;
            // For every wheel rotation, the x-value of the AprilTag on the Camera moves about this much.
            double xChangePerRot = 4.00;



            if (tagProcessor.getDetections().size() > 0) {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);

                telemetry.addData("Id", tag.id);
                telemetry.addLine(String.format("XYZ %6.2f $6.2f %6.2f",tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z));

                telemetry.addData("exposure", exposure.isExposureSupported());

                //A lot of these are optional. Use whatever.
                //telemetry.addData("Center", tag.center);
                //telemetry.addData("roll", tag.ftcPose.roll);
                //telemetry.addData("pitch", tag.ftcPose.pitch);
                telemetry.addData("yaw", tag.ftcPose.yaw);

                //telemetry.addData("range", tag.ftcPose.range);
                //telemetry.addData("bearing", tag.ftcPose.bearing);
                //telemetry.addData("elevation", tag.ftcPose.elevation);


                //double stopWhenClose = tag.ftcPose.range;

                /*if(stopWhenClose < 15){
                    lf.setPower(.3*frontLeftPower);
                    lb.setPower(.3*backLeftPower);
                    rf.setPower(.3*frontRightPower);
                    rb.setPower(.3*backRightPower);
                } */

                //The number of rotations needed by the wheels (strafing) to reach the center of the screen.
                double numRotForCenter = tag.ftcPose.x/xChangePerRot;

                if(tag.ftcPose.x > 0){
                    while(tag.ftcPose.x > 0) {
                        if (gamepad1.left_bumper) {
                            lf.setPower(.3 * frontLeftPower);
                            lb.setPower(-.3 * backLeftPower);
                            rf.setPower(-.3 * frontRightPower);
                            rb.setPower(.3 * backRightPower);
                        }
                    }
                } else if(tag.ftcPose.x < 0){
                    while(tag.ftcPose.x > 0) {
                        if (gamepad1.left_bumper) {
                            lf.setPower(-.3 * frontLeftPower);
                            lb.setPower(.3 * backLeftPower);
                            rf.setPower(.3 * frontRightPower);
                            rb.setPower(-.3 * backRightPower);
                        }
                    }
                }

                if (fixAngle > 0){
                    if(tag.ftcPose.roll > 5){
                        lf.setPower(.25);
                        lb.setPower(.25);
                        rf.setPower(-.25);
                        rb.setPower(-.25);
                    } else if(tag.ftcPose.roll < -5){
                        lf.setPower(-.25);
                        lb.setPower(-.25);
                        rf.setPower(.25);
                        rb.setPower(.25);
                    }
                }
            }



            telemetry.update();

        }
    }
}


