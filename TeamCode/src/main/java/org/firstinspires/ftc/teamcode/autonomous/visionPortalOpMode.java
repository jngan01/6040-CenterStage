package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Processor.VisionPortalProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class visionPortalOpMode extends OpMode{
    private VisionPortalProcessor visionPortalProcessor;
    private VisionPortal visionPortal;

    @Override
    public void init(){
        visionPortalProcessor = new VisionPortalProcessor();
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), visionPortalProcessor);
    }
    @Override
    public void init_loop(){

    }
    @Override
    public void start(){

        visionPortal.stopStreaming();
    }
    @Override
    public void loop(){
        
    }
}
