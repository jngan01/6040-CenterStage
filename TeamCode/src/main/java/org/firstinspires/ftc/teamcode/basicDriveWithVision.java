package org.firstinspires.ftc.teamcode;

import android.provider.ContactsContract;
import android.util.Size;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

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

        waitForStart();


        while (!isStopRequested() && opModeIsActive()) {
           /* //Drive
            lf.setPower(gamepad1.left_stick_y * .75);
            lb.setPower(gamepad1.left_stick_y * .75);
            rf.setPower(gamepad1.left_stick_y * .75);
            rb.setPower(gamepad1.left_stick_y * .75);
        //Strafe
            lf.setPower(gamepad1.left_stick_x * .75);
            lb.setPower(-gamepad1.left_stick_x * .75);
            rf.setPower(-gamepad1.left_stick_x * .75);
            rb.setPower(gamepad1.left_stick_x * .75);
        //Turn
            lf.setPower(gamepad1.left_stick_x * .75);
            lb.setPower(gamepad1.left_stick_x * .75);
            rf.setPower(-gamepad1.left_stick_x * .75);
            rb.setPower(-gamepad1.left_stick_x * .75); */

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
                lf.setPower(.85*frontLeftPower);
                lb.setPower(.85*backLeftPower);
                rf.setPower(.85*frontRightPower);
                rb.setPower(.85*backRightPower);
            }

            float fixAngle = gamepad1.right_trigger;

            if (tagProcessor.getDetections().size() > 0) {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);

                telemetry.addData("Center", tag.center);
                telemetry.addData("Id", tag.id);

                telemetry.addData("x", tag.ftcPose.x);
                telemetry.addData("y", tag.ftcPose.y);
                telemetry.addData("z", tag.ftcPose.z);

                telemetry.addData("roll", tag.ftcPose.roll);
                telemetry.addData("pitch", tag.ftcPose.pitch);
                telemetry.addData("yaw", tag.ftcPose.yaw);

                telemetry.addData("range", tag.ftcPose.range);
                telemetry.addData("bearing", tag.ftcPose.bearing);
                telemetry.addData("elevation", tag.ftcPose.elevation);

                if(fixAngle > 0){
                    if(tag.ftcPose.roll > 0){
                        lf.setPower(.5);
                        lb.setPower(.5);
                        rf.setPower(-.5);
                        rb.setPower(-.5);
                    } else if(tag.ftcPose.roll < 0){
                        lf.setPower(-.5);
                        lb.setPower(-.5);
                        rf.setPower(.5);
                        rb.setPower(.5);
                    }
                }
            }


            telemetry.update();

        }
    }
}


