package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class AutoBeanBlueTest extends LinearOpMode {
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private CRServo clawIntake;
    private DcMotor fourBar;
    private DcMotor topLift;
    private DcMotor bottomLift;

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        clawIntake = hardwareMap.crservo.get("clawIntake");
        fourBar = hardwareMap.dcMotor.get("fourBar");
        topLift = hardwareMap.dcMotor.get("liftTop");
        bottomLift = hardwareMap.dcMotor.get("liftBottom");
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();

        clawIntake.setPower(-.25);
        strafe(-.25);
        sleep(3750);
        strafe(0);
        fourBar.setPower(-1);
        sleep(2000);
        leftFront.setPower(.25 );
        leftBack.setPower(.25);
        rightFront.setPower(.25);
        rightBack.setPower(.25);
        sleep(350);
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        sleep(1000);
        fourBar.setPower(.5);
        sleep(1000);
        clawIntake.setPower(.25);
        sleep(1000);
        fourBar.setPower(0);
        leftFront.setPower(-.25);
        leftBack.setPower(-.25);
        rightFront.setPower(-.25);
        rightBack.setPower(-.25);
        sleep(1000);
        strafe(-1);
        sleep(1500);
        strafe(0);

    }
    public void strafe(double direction) {

        leftFront.setPower(-1 * direction);
        leftBack.setPower(1 * direction);
        rightFront.setPower(1 * direction);
        rightBack.setPower(-1 * direction);

    }
}


