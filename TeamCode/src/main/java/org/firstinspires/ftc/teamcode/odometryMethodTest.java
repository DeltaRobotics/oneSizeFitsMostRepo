package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
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
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        robotHardware robot = new robotHardware(hardwareMap);

        robot.resetDriveEncoders();

        double x = 24;
        double y = 0;
        double distance = Math.hypot(x - robot.GlobalX, y - robot.GlobalY);

        telemetry.addData("x",robot.GlobalX);
        telemetry.addData("y",robot.GlobalY);
        telemetry.addData("heading",robot.GlobalHeading);
        telemetry.addData("distance",distance);
        telemetry.update();

        waitForStart();


        while(true) {

            x = 40;
            y = 0;
            distance = Math.hypot(x - robot.GlobalX, y - robot.GlobalY);

            robot.goToPosSingle(x, y, Math.toRadians(0), Math.toRadians(0));
            telemetry.addData("x",robot.GlobalX);
            telemetry.addData("y",robot.GlobalY);
            telemetry.addData("heading",robot.GlobalHeading);
            telemetry.addData("distance",distance);
            telemetry.update();

        }










       //while (opModeIsActive()) {

       //    robot.mecanumDrive(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, 1);

       //    robot.refresh(robot.odometers);

       //    telemetry.addData("X", robot.GlobalX);
       //    telemetry.addData("Y", robot.GlobalY);
       //    telemetry.addData("Heading", Math.toDegrees(robot.GlobalHeading));

       //    telemetry.addData("motorRFPower", robot.motorRF.getPower());
       //    telemetry.addData("motorRBPower", robot.motorRB.getPower());
       //    telemetry.addData("motorLBPower", robot.motorLB.getPower());
       //    telemetry.addData("motorLFPower", robot.motorLF.getPower());

       //    telemetry.update();

       //}
    }
}