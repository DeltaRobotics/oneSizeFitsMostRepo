package org.firstinspires.ftc.teamcode;


import static java.lang.System.nanoTime;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.MovingStatistics;

/**

 this is from ulimate goal.
 it is an old dashboard based pid tuner for the built in motor PID
 try using it with the new PID
 https://github.com/DeltaRobotics-FTC/DR_20-21SDK6.1/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/VelocityTuning/velocityPIDFTuner.java
 */

@Config
@TeleOp
public class PIDTest2 extends LinearOpMode
{
    DcMotorEx motor;

    //do not put equasions in for PIDF variables, enter the answers to the math.
    //the numbers from the equasions are starting points
    public static double F = 0.75; // = 32767 / maxV      (do not edit from this number)
    public static double P = 0.01; // = 0.1 * F           (raise till real's apex touches Var apex)
    public static double I = 0.01;// = 0.1 * P           (fine ajustment of P)
    public static double D = 0.0001; // = 0                     (raise to reduce ocolation)

    double wheelSpeed = 1700;
    double speed = 0;
    double speed2 = 0;
    double speed3 = 0;

    double currentTime = 0;
    double time = 0;
    double lastTime = 0;
    double error = 0;
    double previousError = 0;
    double totalError = 0;
    double minIntegral = -1.0;
    double maxIntegral = 1.0;
    double motorPower = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        motor = hardwareMap.get(DcMotorEx.class, "SlideLeft2");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ElapsedTime timer = new ElapsedTime();

        /**
         * look into item bellow
         */
        //this.velocityEstimates = new MovingStatistics(100);

        waitForStart();

        while (opModeIsActive()) {

            //motor.setVelocityPIDFCoefficients( P , I , D , F);

            timer.reset();

            //((DcMotorEx) motor).setVelocity(wheelSpeed);


            while (timer.seconds() < 5) {
                motor.setPower(-PID(wheelSpeed, speed));
                speed3 = speed2;
                speed2 = speed;
                speed = motor.getVelocity();
                telemetry.addData("wheelSpeedVar", wheelSpeed);
                telemetry.addData("wheelSpeedReal", ((speed3 + speed2 +speed)/3));
                telemetry.addData("time", time);
                telemetry.addData("current time", currentTime);
                telemetry.addData("last time", lastTime);
                telemetry.addData("error", error);
                telemetry.addData("total error", totalError);
                telemetry.addData("previous error", previousError);
                telemetry.addData("motor power", motorPower);
                telemetry.update();
            }

            //((DcMotorEx) motor).setVelocity(0);
            motor.setPower(0);


            while (timer.seconds() < 20) {
                telemetry.addData("wheelSpeedVar", wheelSpeed);
                telemetry.addData("wheelSpeedReal", ((DcMotorEx) motor).getVelocity());
                telemetry.addData("time", time);
                telemetry.addData("current time", currentTime);
                telemetry.addData("last time", lastTime);
                telemetry.addData("error", error);
                telemetry.addData("total error", totalError);
                telemetry.addData("previous error", previousError);
                telemetry.update();
            }
        }
    }

    public double PID(double target, double current){
        previousError = error;
        error = target - current;
        lastTime = currentTime;
        currentTime = (double) System.nanoTime()/1E9;
        time = currentTime - lastTime;
        totalError += time * error;
        totalError = totalError < minIntegral ? minIntegral: Math.min(maxIntegral, totalError);

        motorPower = (P * error) + (I * totalError) + (D * (error - previousError) / time) + F;
        return motorPower;
    }
}