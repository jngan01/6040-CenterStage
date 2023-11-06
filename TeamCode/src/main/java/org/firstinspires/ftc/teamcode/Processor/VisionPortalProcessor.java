package org.firstinspires.ftc.teamcode.Processor;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Rect;
import org.opencv.core.Mat;
public class VisionPortalProcessor implements VisionProcessor {
    // Creation of a rectangle with camera cords and size.
    public Rect rect = new Rect(20, 20, 50, 50);

    @Override
    public  void  init(int width, int height, CameraCalibration calibration){

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos){
        return null;
    }

    // Conversion from OpenCV camera rectangle to an android.graphics.Rect.
    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx){
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreebHeight, float scaleBmpPxToCanvasPx,
                            float scaleCanvasDensity, Object userContext){
        //Setting up the actual drawing of the rectangle
        Paint rectPaint = new Paint();
        rectPaint.setColor(Color.RED);
        rectPaint.setStyle(Paint.Style.STROKE);
        rectPaint.setStrokeWidth(scaleCanvasDensity * 4);

        canvas.drawRect(makeGraphicsRect(rect, scaleBmpPxToCanvasPx), rectPaint);
    }
}
