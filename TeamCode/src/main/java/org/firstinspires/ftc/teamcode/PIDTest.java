package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="PIDTest")

public class PIDTest extends LinearOpMode {
    public DcMotor motor = null;
    int speedNew = 0;
    int speedOld = 0;
    int speed = 0;
    double oldTime = 0.0;
    double currentTime = 0.0;
    public double power = 1;

    public void runOpMode() throws InterruptedException{
        motor = hardwareMap.dcMotor.get("SlideLeft2");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Only if you are using ftcdashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        waitForStart();

        while(opModeIsActive()) {

            motor.setPower(power);

            speedNew = motor.getCurrentPosition();
            speed = speedNew - speedOld;
            speedOld = speedNew;

            currentTime = getRuntime();

            telemetry.addData("Motor speed", speed /(currentTime - oldTime));
            //add telemetry speed and time
            telemetry.update();

            oldTime = currentTime;
        }
    }
}
