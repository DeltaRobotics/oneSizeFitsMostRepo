package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="mecanumDrive")
//@Disabled

public class mecanumDrive extends LinearOpMode{


    public DcMotor motorRF = null;
    public DcMotor motorLF = null;
    public DcMotor motorRB = null;
    public DcMotor motorLB = null;

    public void runOpMode() throws InterruptedException {


        motorRF = hardwareMap.dcMotor.get("motorRF");
        motorLF = hardwareMap.dcMotor.get("motorLF");
        motorRB = hardwareMap.dcMotor.get("motorRB");
        motorLB = hardwareMap.dcMotor.get("motorLB");

        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorLF.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLB.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            motorRF.setPower((((-gamepad1.right_stick_y - gamepad1.right_stick_x) * 1) - (gamepad1.left_stick_x * 1)));
            motorRB.setPower((((-gamepad1.right_stick_y + gamepad1.right_stick_x) * 1) - (gamepad1.left_stick_x * 1)));
            motorLB.setPower((((-gamepad1.right_stick_y - gamepad1.right_stick_x) * 1) + (gamepad1.left_stick_x * 1)));
            motorLF.setPower((((-gamepad1.right_stick_y + gamepad1.right_stick_x) * 1) + (gamepad1.left_stick_x * 1)));

            telemetry.addData("motorRFPower", motorRF.getPower());
            telemetry.addData("motorRBPower", motorRB.getPower());
            telemetry.addData("motorLBPower", motorLB.getPower());
            telemetry.addData("motorLFPower", motorLF.getPower());

            telemetry.update();

        }
    }



}