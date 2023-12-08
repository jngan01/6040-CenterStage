package org.firstinspires.ftc.teamcode.pastSeasons;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;


@Disabled
public class FieldCentricOdometry extends LinearOpMode {

    private DcMotor leftDrive, rightDrive, frontDrive, backDrive;
    private BNO055IMU imu;

    private double robotX = 0;
    private double robotY = 0;
    private double robotAngle = 0;

    @Override
    public void runOpMode() {
        // Initialize your motors, encoders, and IMU here
        leftDrive = hardwareMap.get(DcMotor.class, "lf");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        frontDrive = hardwareMap.get(DcMotor.class, "front_drive");
        backDrive = hardwareMap.get(DcMotor.class, "back_drive");

        // Initialize and configure your IMU (Inertial Measurement Unit)
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.mode = BNO055IMU.SensorMode.IMU;
        imu.initialize(imuParameters);

        // Other initialization code goes here

        waitForStart();

        while (opModeIsActive()) {
            // Odometry update code goes here
            updateOdometry();

            // Field-centric drive code goes here
            double driveX = gamepad1.left_stick_x;
            double driveY = -gamepad1.left_stick_y; // Invert for gamepad orientation
            double turn = gamepad1.right_stick_x;

            // Convert field-centric vectors to robot-centric
            double temp = driveX * Math.cos(robotAngle) + driveY * Math.sin(robotAngle);
            driveY = -driveX * Math.sin(robotAngle) + driveY * Math.cos(robotAngle);
            driveX = temp;

            // Calculate motor powers based on drive and turn inputs
            double frontLeftPower = driveY + turn + driveX;
            double frontRightPower = driveY - turn - driveX;
            double backLeftPower = driveY + turn - driveX;
            double backRightPower = driveY - turn + driveX;

            // Set motor powers (adjust as needed)
            leftDrive.setPower(frontLeftPower);
            rightDrive.setPower(frontRightPower);
            frontDrive.setPower(backLeftPower);
            backDrive.setPower(backRightPower);

            // Other robot functionality code goes here

            telemetry.addData("X Position", robotX);
            telemetry.addData("Y Position", robotY);
            telemetry.addData("Robot Angle", robotAngle);
            telemetry.update();
        }
    }

    private void updateOdometry() {
        // Implement your odometry update logic here
        // Use encoders and/or other sensors to calculate robot position and angle
        // Update robotX, robotY, and robotAngle variables
    }
}
