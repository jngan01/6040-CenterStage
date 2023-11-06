package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class OpenCVTest extends OpMode {
    OpenCvWebcam webcam1 = null;

    @Override
    public void init(){

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("CameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        webcam1.setPipeline(new examplePipeline());

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam1.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("OpenCVCamera not working");
            }
        });
    }
    @Override
    public void loop(){

    }
    class examplePipeline extends OpenCvPipeline{

        Mat YCbCr = new Mat();
        Mat leftCrop;
        Mat middleCrop;
        Mat rightCrop;
        double leftavgfin;
        double middleavgfin;
        double rightavgfin;
        Mat outPut = new Mat();
        Scalar rectColor = new Scalar(255.0, 0.0, 0.0);

        Rect leftRect = new Rect(60, 200, 100, 100);
        Rect middleRect = new Rect(250, 200, 100, 100);
        Rect rightRect = new Rect(450, 200, 100, 100);

        public void init(Mat YCbCr){
            leftCrop = YCbCr.submat(leftRect);
            middleCrop = YCbCr.submat(middleRect);
            rightCrop = YCbCr.submat(rightRect);

            Scalar leftavg = Core.mean(leftCrop);
            Scalar middleavg = Core.mean(middleCrop);
            Scalar rightavg = Core.mean(rightCrop);

            leftavgfin = leftavg.val[0];
            middleavgfin = middleavg.val[0];
            rightavgfin = rightavg.val[0];

            Core.extractChannel(leftCrop, leftCrop, 2);
            Core.extractChannel(middleCrop, middleCrop, 2);
            Core.extractChannel(rightCrop, rightCrop, 2);
        }
        public Mat processFrame(Mat input){
            Imgproc.cvtColor(input,YCbCr,Imgproc.COLOR_RGB2YCrCb);
            telemetry.addLine("pipeline running");

            input.copyTo(outPut);

            Imgproc.rectangle(outPut, leftRect, rectColor, 2);
            Imgproc.rectangle(outPut, middleRect, rectColor, 2);
            Imgproc.rectangle(outPut, rightRect, rectColor, 2);

            if (leftavgfin > rightavgfin && leftavgfin > middleavgfin){
                telemetry.addLine("Left");
            } else if(rightavgfin > leftavgfin && rightavgfin > middleavgfin){
                telemetry.addLine("right");
            } else if(middleavgfin > rightavgfin && middleavgfin > leftavgfin){
                telemetry.addLine("Middle");
            } else{
                telemetry.addLine("Nada");
            }

            return(outPut);

        }
    }
}
