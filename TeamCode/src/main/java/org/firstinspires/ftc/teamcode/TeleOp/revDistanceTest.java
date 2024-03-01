package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class revDistanceTest extends LinearOpMode {

    private DistanceSensor distanceSensorLeft;
    private DistanceSensor distanceSensorRight;
    private Servo rightIntake;
    private Servo leftIntake;

    @Override
    public void runOpMode() throws InterruptedException {

        distanceSensorLeft = hardwareMap.get(DistanceSensor.class, "distanceSensorLeft");
        distanceSensorRight = hardwareMap.get(DistanceSensor.class, "distanceSensorRight");
        leftIntake = hardwareMap.get(Servo.class, "leftIntake");
        rightIntake = hardwareMap.get(Servo.class, "rightIntake");

        leftIntake.setPosition(0);
        rightIntake.setPosition(0);
        boolean pixelScannerOn = false;

        if(distanceSensorLeft.getDistance(DistanceUnit.CM) > 0) {
            telemetry.addLine("Left Distance Sensor On");
            telemetry.addData("Range:", String.format("%.01f mm", distanceSensorLeft.getDistance(DistanceUnit.CM)));
        }

        if(distanceSensorRight.getDistance(DistanceUnit.CM) > 0) {
            telemetry.addLine("Right Distance Sensor On");
            telemetry.addData("Range:", String.format("%.01f mm", distanceSensorLeft.getDistance(DistanceUnit.CM)));
        waitForStart();



        }
        while (!isStopRequested() && opModeIsActive()) {

            if(gamepad1.a){
                    pixelScannerOn = true;
                leftIntake.setPosition(1);
                rightIntake.setPosition(0);

            }
            if(gamepad1.b){
                pixelScannerOn = false;
            }

            if(pixelScannerOn == true){
                if(distanceSensorLeft.getDistance(DistanceUnit.CM) <= 5.2 || distanceSensorRight.getDistance(DistanceUnit.CM) <= 5.2){

                    leftIntake.setPosition(0);
                    rightIntake.setPosition(1);
                    pixelScannerOn = false;
                    telemetry.update();
                }
            }

            telemetry.addData("Range:", String.format("%.01f cm", distanceSensorLeft.getDistance(DistanceUnit.CM)));
            telemetry.addData("Range:", String.format("%.01f cm", distanceSensorRight.getDistance(DistanceUnit.CM)));
            telemetry.update();
        }
    }

}
