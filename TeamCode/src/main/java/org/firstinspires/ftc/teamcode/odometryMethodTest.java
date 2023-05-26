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

        DcMotor[] odometers = new DcMotor[3];
        {
            odometers[0] = robot.leftEncoder;
            odometers[1] = robot.rightEncoder;
            odometers[2] = robot.perpendicularEncoder;
        }
        DcMotor[] drive = new DcMotor[4];
        {
            drive[0] = robot.motorRF;
            drive[1] = robot.motorRB;
            drive[2] = robot.motorLB;
            drive[3] = robot.motorLF;
        }

        VoltageSensor ControlHub_VoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        waitForStart();



/*
        robot.goToPos(odometers, drive, ControlHub_VoltageSensor, 12, 12, Math.toRadians(-90), .25, 1, .5, Math.toRadians(1), Math.toRadians(0));
        robot.wait(1000, odometers);
        robot.goToPos(odometers, drive, ControlHub_VoltageSensor, 12, 0, Math.toRadians(-90), 1, .5, .5, Math.toRadians(1), Math.toRadians(180));
        robot.wait(1000, odometers);
        robot.goToPos(odometers, drive, ControlHub_VoltageSensor, 0, 0, Math.toRadians(0), .75, 1, .5, Math.toRadians(1), Math.toRadians(90));
        robot.wait(1000, odometers);
        robot.goToPos(odometers, drive, ControlHub_VoltageSensor, 0, 12, Math.toRadians(0), .5, .5, .5, Math.toRadians(1), Math.toRadians(90));
        robot.wait(1000, odometers);
        robot.goToPos(odometers, drive, ControlHub_VoltageSensor, 12, 12, Math.toRadians(45), 1, .75, .5, Math.toRadians(1), Math.toRadians(0));
        robot.wait(1000, odometers);
        robot.goToPos(odometers, drive, ControlHub_VoltageSensor, 12, 0, Math.toRadians(135), .25, 1, .5, Math.toRadians(1), Math.toRadians(0));
        robot.wait(1000, odometers);
        robot.goToPos(odometers, drive, ControlHub_VoltageSensor, 0, 0, Math.toRadians(180), 1, .5, .5, Math.toRadians(1), Math.toRadians(0));
        robot.wait(1000, odometers);
        robot.goToPos(odometers, drive, ControlHub_VoltageSensor, 0, 0, Math.toRadians(0), .25, .5, .5, Math.toRadians(1), Math.toRadians(0));
*/
        robot.goToPos(odometers, drive, ControlHub_VoltageSensor, -44.5, 0, Math.toRadians(125), .7, .7, 1, Math.toRadians(3), Math.toRadians(-180));












        while (opModeIsActive()) {

            robot.mecanumDrive(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, 1);

            robot.refresh(odometers);

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