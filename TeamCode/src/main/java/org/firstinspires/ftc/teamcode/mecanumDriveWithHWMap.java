package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="mecanumDriveWithHWMap")
//@Disabled

public class mecanumDriveWithHWMap extends LinearOpMode{
    public void runOpMode() throws InterruptedException {

        robotHardware robot = new robotHardware(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            robot.mecanumDrive(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x);

            telemetry.addData("motorRFPower", robot.motorRF.getPower());
            telemetry.addData("motorRBPower", robot.motorRB.getPower());
            telemetry.addData("motorLBPower", robot.motorLB.getPower());
            telemetry.addData("motorLFPower", robot.motorLF.getPower());

            telemetry.update();
        }
    }
}