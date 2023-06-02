package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robotHardware;

@TeleOp(name="odometryMethodTest")
//@Disabled

public class odometryMethodTest extends LinearOpMode{

    double speed = 1;
    double zScale = 1;

    public void runOpMode() throws InterruptedException {

        robotHardware robot = new robotHardware(hardwareMap);

        robot.resetDriveEncoders();

        waitForStart();




        robot.goToPos(12, 12, Math.toRadians(-90), Math.toRadians(0));
        robot.wait(1000, robot.odometers);
        robot.goToPos(12, 0, Math.toRadians(-90), Math.toRadians(180));
        robot.wait(1000, robot.odometers);
        robot.goToPos(0, 0, Math.toRadians(0), Math.toRadians(90));
        robot.wait(1000, robot.odometers);
        robot.goToPos(0, 12, Math.toRadians(0), Math.toRadians(90));
        robot.wait(1000, robot.odometers);
        robot.goToPos(12, 12, Math.toRadians(45), Math.toRadians(0));
        robot.wait(1000, robot.odometers);
        robot.goToPos(12, 0, Math.toRadians(135), Math.toRadians(0));
        robot.wait(1000, robot.odometers);
        robot.goToPos(0, 0, Math.toRadians(180), Math.toRadians(0));
        robot.wait(1000, robot.odometers);
        robot.goToPos(0, 0, Math.toRadians(0), Math.toRadians(0));

        //robot.goToPos(odometers, drive, ControlHub_VoltageSensor, -44.5, 0, Math.toRadians(125), .7, .7, 1, Math.toRadians(3), Math.toRadians(-180));











        while (opModeIsActive()) {

            robot.mecanumDrive(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, 1);

            robot.refresh(robot.odometers);

            telemetry.addData("X", robot.GlobalX);
            telemetry.addData("Y", robot.GlobalY);
            telemetry.addData("Heading", Math.toDegrees(robot.GlobalHeading));

            telemetry.addData("motorRFPower", robot.motorRF.getPower());
            telemetry.addData("motorRBPower", robot.motorRB.getPower());
            telemetry.addData("motorLBPower", robot.motorLB.getPower());
            telemetry.addData("motorLFPower", robot.motorLF.getPower());

            telemetry.update();

        }
    }
}