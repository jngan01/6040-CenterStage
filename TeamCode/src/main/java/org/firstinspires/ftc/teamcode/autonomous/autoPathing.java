package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;

@Autonomous
public class autoPathing extends LinearOpMode {

    private DcMotor arm;
    private Servo rotator;
    private Servo wrist;
    private Servo clamp;

    @Override
    public void runOpMode() throws InterruptedException {

        arm = hardwareMap.get(DcMotor.class, "arm");
        rotator = hardwareMap.get(Servo.class, "rotator");
        wrist = hardwareMap.get(Servo.class, "wrist");
        clamp = hardwareMap.get(Servo.class, "clamp");

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER );

        Hardware robot = new Hardware(this);
        robot.init();

        waitForStart();

        clamp.setPosition(0);
        wrist.setPosition(1);
        rotator.setPosition(0);
        sleep(250);

        telemetry.addLine("Driving forward");
        telemetry.update();
        robot.setDrivePower(-0.5, -0.5, -0.5, -0.5);
        sleep(1000);
        robot.setDrivePower(0, 0, 0,0);

        telemetry.addLine("Turn 90 degrees");
        telemetry.update();
        // pretty much 90 degrees?
        robot.setDrivePower(-0.5, -0.5, 0.5, 0.5);
        sleep(950);

        telemetry.addLine("Push pixel backwards to line");
        telemetry.update();
        robot.setDrivePower(0.5, 0.5, 0.5, 0.5);
        sleep(100);

        robot.setDrivePower(-0.5, -0.5, -0.5,-0.5);
        sleep(400);

        telemetry.addLine("Turn 180 degrees");
        telemetry.update();

        robot.setDrivePower(0.5, 0.5, -0.5, -0.5);
        sleep(1900);

        clamp.setPosition(1);
        wrist.setPosition(.4);
        arm.setPower(.5);
        sleep(2000);
        arm.setPower(0);
    }
}
