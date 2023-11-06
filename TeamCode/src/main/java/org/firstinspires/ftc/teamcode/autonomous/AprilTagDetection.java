package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

import java.util.concurrent.TimeUnit;

@Autonomous
public class AprilTagDetection extends LinearOpMode {

    private DcMotor lf;
    private DcMotor lb;
    private DcMotor rf;
    private DcMotor rb;

    private int lfPos;
    private int lbPos;
    private int rfPos;
    private int rbPos;

    @Override
    public void runOpMode() throws InterruptedException {

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

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lfPos = 0;
        lbPos = 0;
        rfPos = 0;
        rbPos = 0;

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
        ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);
        exposure.setMode(ExposureControl.Mode.Manual);
        exposure.setExposure(15, TimeUnit.MILLISECONDS);

        GainControl gain = visionPortal.getCameraControl((GainControl.class));
        gain.setGain(100);
        waitForStart();


        while (!isStopRequested() && opModeIsActive()) {

            if (tagProcessor.getDetections().size() > 0) {
                org.firstinspires.ftc.vision.apriltag.AprilTagDetection tag = tagProcessor.getDetections().get(0);

                telemetry.addData("Id", tag.id);
                telemetry.addLine(String.format("XYZ %6.2f $6.2f %6.2f", tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z));

                telemetry.addData("exposure", exposure.isExposureSupported());

                //A lot of these are optional. Use whatever.
                //telemetry.addData("Center", tag.center);
                //telemetry.addData("roll", tag.ftcPose.roll);
                //telemetry.addData("pitch", tag.ftcPose.pitch);
                telemetry.addData("yaw", tag.ftcPose.yaw);

                //telemetry.addData("range", tag.ftcPose.range);
                //telemetry.addData("bearing", tag.ftcPose.bearing);
                //telemetry.addData("elevation", tag.ftcPose.elevation);

                if (tag.id == 4) {

                    drive(0, 0, 0, 0, 0);

                }

            }
        }

        while (opModeIsActive() && lf.isBusy() && lb.isBusy() && rf.isBusy() && rb.isBusy()) {
            idle();
        }
    }
    private void drive(int lfTarget, int lbTarget, int rfTarget, int rbTarget, double speed) {
        lfPos += lfTarget;
        lbPos += lbTarget;
        rfPos += rfTarget;
        rbPos += rbTarget;

        lf.setTargetPosition(lfPos);
        lb.setTargetPosition(lbPos);
        rf.setTargetPosition(rfPos);
        rb.setTargetPosition(rbPos);

        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lf.setPower(speed);
        lb.setPower(speed);
        rf.setPower(speed);
        rb.setPower(speed);
    }

    }


