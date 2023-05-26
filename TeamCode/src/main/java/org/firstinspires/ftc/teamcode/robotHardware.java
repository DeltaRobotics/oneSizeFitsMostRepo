package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by User on 10/1/2022.
 */

public class robotHardware extends LinearOpMode
{
    //drive motors
    public DcMotor motorRF = null;
    public DcMotor motorLF = null;
    public DcMotor motorRB = null;
    public DcMotor motorLB = null;

    //odometry encoder objects
    public DcMotor leftEncoder = null;
    public DcMotor rightEncoder = null;
    public DcMotor perpendicularEncoder = null;

    HardwareMap hwMap = null;

    public robotHardware(HardwareMap ahwMap)
    {
        //dive motors
        motorRF = ahwMap.dcMotor.get("motorRF");
        motorLF = ahwMap.dcMotor.get("motorLF");
        motorRB = ahwMap.dcMotor.get("motorRB");
        motorLB = ahwMap.dcMotor.get("motorLB");

        //drive motors and odometry encoders
        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorLF.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLB.setDirection(DcMotorSimple.Direction.REVERSE);

        motorRF.setPower(0);
        motorLF.setPower(0);
        motorRB.setPower(0);
        motorLB.setPower(0);

        //odometry init (use the motors objects that the odometers are plugged into)
        leftEncoder = motorLF;
        rightEncoder = motorRB;
        perpendicularEncoder = motorLB;
    }

    public void mecanumDrive(double forward, double strafe, double heading, double speed){
        motorRF.setPower((((forward - strafe) * 1) - (heading * 1)) * speed);
        motorRB.setPower((((forward + strafe) * 1) - (heading * 1)) * speed);
        motorLB.setPower((((forward - strafe) * 1) + (heading * 1)) * speed);
        motorLF.setPower((((forward + strafe) * 1) + (heading * 1)) * speed);
    }

    public void resetDriveEncoders()
    {
        motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * how to use this code,
     *
     * the cordinant system is a rotated cordinate system with X being forward.
     *
     *       ^ x+
     *       |
     * y+    |
     * <-----O
     *
     *
     *
     * the drive code is written for a mecanum drive, but the localizer will work with any drive train.
     * as long as the odometry pods are set up correctly.
     *
     * there should be 2 forward facing odometry wheels, and one sideways odometry wheel.
     * illistrated bellow:
     *
     *    /--------------\
     *    |     ____     |
     *    |     ----     |
     *    | ||        || |
     *    | ||        || |
     *    |              |
     *    |              |
     *    \--------------/
     *
     *
     *
     *
     * to begin using the code
     * ensure the following lines are in your hardware map:
     * ---------------------------------------------------------------
     * motorRF = ahwMap.dcMotor.get("motorRF");
     * motorLF = ahwMap.dcMotor.get("motorLF");
     * motorRB = ahwMap.dcMotor.get("motorRB");
     * motorLB = ahwMap.dcMotor.get("motorLB");
     *
     * motorLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
     * motorLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
     * motorRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
     * motorRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
     *
     * leftEncoder = motorLF;
     * rightEncoder = motorRF;
     * perpendicularEncoder = motorRB;
     * ---------------------------------------------------------------
     * replace "____Encoder = motor__" with the correct motor (the same motor from the port that the odometry encoder shares)
     *
     **//* the blanks here refer to the words 'left', 'right', and 'perpendicular'*//**
 *
 * replace "odometryRobotHardware robot = new odometryRobotHardware(hardwareMap);" with the chosen hardware map
 *
 *
 **//* the blanks here refer to a program that will be made in the future*/

    /**
     * using ____, tune the values accordingly
     * 1. enter the radius of the odometer wheel as R
     *      to tune:
     *      - push the robot straight forward 100".
     *      - if the distance is not close to 100, adjust R
     *      - repeat till the distance read is close to 100"
     *
     *
     * 2. enter the distance between the 2 forward facing encoder wheels in inches as L.
     *      to tune:
     *      - spin 10 times
     *      - if the angle does not read around 0, change L up or down a small amount.
     *      - repeat till the angle reads around 0
     *
     *
     * 3. enter the distance from the middle of a forward facing encoder wheel to the middle of the sideways encoder wheel as B
     *      to tune:
     *      - spin 10 times
     *  *   - if the y distance does not read around 0, change B up or down a small amount.
     *  *   - repeat till the y distance reads around 0
     *
     *
     *
     *  notes:
     * when using the code in auto the refresh() method must be constantly updated
     * use a structure similar to bellow:
     * ---------------------------------------------------------------
     * odometry.goToPos(...);
     * odometry.wait(...);
     * move a servo
     * odometry.wait(...);
     * odometry.goToPos(...);
     * ect.
     * ---------------------------------------------------------------
     * this use of odometry.wait(...); allows the odometry to continue to update. compared to a sleep(...); which pauses all the code for the duration of the sleep.
     */

    //odometry constants (tune these)
    double L = 11.875;   //distance between left and right odometers (in inches)
    double B = 1.125;   //distance from center of left/right encoders to the perpendicular encoder (in inches)
    double R = .7514;   //wheel radius (in inches)
    double N = 8192;  //encoder ticks per revoluton
    double inPerTick = 2.0 * Math.PI * R / N;

    //changes starting location (in inches)
    public double GlobalX = 0;
    public double GlobalY = 0;
    public double GlobalHeading = 0;

    //track encoder values between loops
    private int currentRightPos = 0;
    private int currentLeftPos = 0;
    private int currentPerpendicularPos = 0;
    private int oldRightPos = 0;
    private int oldLeftPos = 0;
    private int oldPerpendicularPos = 0;

    /**
     * refresh() is the core of the odometry code.
     * calling this method will recalculate the location of the bot, but must be updating regularily inside a loop
     * it will save public values which can be accessed to identify the global position of the robot.
     *
     * enter motors into the array 0.left, 1.right, 2.perpendicular
     *
     * for a good explination of the math behind odometry watch this video:
     * https://www.youtube.com/watch?v=Av9ZMjS--gY
     */

    public void refresh(DcMotor[] odometers)
    {

        //record last loop's encoder reading
        oldRightPos = currentRightPos;
        oldLeftPos = currentLeftPos;
        oldPerpendicularPos = currentPerpendicularPos;

        //record a new encoder reading this loop
        currentRightPos = odometers[0].getCurrentPosition();
        currentLeftPos = odometers[1].getCurrentPosition();
        currentPerpendicularPos = -odometers[2].getCurrentPosition();

        //find the delta encoder values of each encoder
        int dn1 = currentLeftPos - oldLeftPos;
        int dn2 = currentRightPos - oldRightPos;
        int dn3 = currentPerpendicularPos - oldPerpendicularPos;

        //find the delta of x,y,heading reletive to the robot
        double dtheta = inPerTick * (dn2 - dn1) / L;
        double dx = inPerTick * (dn1 + dn2) / 2.0;
        double dy = inPerTick * (dn3 - (dn2 - dn1) * B / L);

        //add the robots movement this loop to the global location
        //double theta = (dtheta / 2.0);
        GlobalHeading += dtheta;
        GlobalX += dx * Math.cos(GlobalHeading) - dy * Math.sin(GlobalHeading);
        GlobalY -= dx * Math.sin(GlobalHeading) + dy * Math.cos(GlobalHeading);
        //todo fix here if wrong


        //makes heading 180 to -180
        angleWrapRad(GlobalHeading);
    }

    // used to mantain angle values between Pi and -Pi
    public double angleWrapRad(double angle)
    {
        while (angle > Math.PI)
        {
            angle -= Math.PI * 2;
        }
        while (angle < -Math.PI)
        {
            angle += Math.PI * 2;
        }

        return angle;
    }

    //use instead of sleep() in autonomus to keep the location updating
    public void wait(double waitTime, DcMotor[] odometers)
    {
        ElapsedTime time = new ElapsedTime();

        while (time.milliseconds() <= waitTime)
        {
            refresh(odometers);
        }
    }

    /**
     * this method is the key to using odometry
     * by imputing a location to drive to the robot will calculate an efficient path to the target.
     * if the robot is interfered with, it will recalculate and adjust accordingly
     *
     * at the beginning of the drive the robot will face the target (because mecanums are faster forward than sideways)
     * when it gets within a set distance of the target it will begin turning toward its desired final orientation
     * when it is within its accuracy requirements for the move it will exit the loop and set the motor powers to 0
     *
     * this code is pulled from the basis of pure pursuit.
     * for a better understanding of pure pursuit and if someone wants to improve this code, look into learning how the rest of gluten free's code works here:
     * https://www.youtube.com/watch?v=3l7ZNJ21wMo (5 parts)
     * the code below uses the code explains in parts 1 & 2
     *
     * for the arrays; odometers should be 0.left, 1.right, 2.perpendicular
     * drive motors should be 0.Right front, 1.left front, 2.left back, 3.right back
     *
     * this version will loop until the desired location is reached and then move on
     */
    public void goToPos(DcMotor[] odometers, DcMotor[] drive, VoltageSensor ControlHub_VoltageSensor, double x, double y, double finalAngle, double moveSpeed, double turnSpeed, double moveAccuracy, double angleAccuracy, double followAngle)
    {
        //bring in the encoder and motor objects
        //odometryRobotHardware robot = new odometryRobotHardware(hardwareMap);

        //while loop makes the code keep running till the desired location is reached. (within the accuracy constraints)
        while(Math.abs(x-GlobalX) > moveAccuracy || Math.abs(y-GlobalY) > moveAccuracy || Math.abs(angleWrapRad(finalAngle - GlobalHeading)) > angleAccuracy) {

            //update odometry location
            refresh(odometers);

            double voltComp = (14.0/ControlHub_VoltageSensor.getVoltage()) * (11.0/14.0);

            //math to calculate distances to the target
            double distanceToTarget = Math.hypot(x - GlobalX, y - GlobalY);
            double absoluteAngleToTarget = Math.atan2(x - GlobalX, y - GlobalY);
            double reletiveAngleToTarget = angleWrapRad(absoluteAngleToTarget - GlobalHeading-Math.toRadians(90));
            double reletiveXToTarget = Math.cos(reletiveAngleToTarget) * distanceToTarget;
            double reletiveYToTarget = Math.sin(reletiveAngleToTarget) * distanceToTarget;

            //slow down ensures the robot does not over shoot the target
            double slowDown = Range.clip(Math.log(distanceToTarget +1) / 4, .125, moveSpeed);

            //calculate the vector powers for the mecanum math
            double movementXpower = (reletiveXToTarget / (Math.abs(reletiveXToTarget) + Math.abs(reletiveYToTarget))) * slowDown;
            double movementYpower = (reletiveYToTarget / (Math.abs(reletiveYToTarget) + Math.abs(reletiveXToTarget))) * slowDown * 1.2;

            //when far away from the target the robot will point at the target to get there faster.
            //at the end of the movement the robot will begin moving toward the desired final angle
            double movementTurnPower;
            double reletiveTurnAngle;
            if (distanceToTarget > 6) {
                reletiveTurnAngle = angleWrapRad(reletiveAngleToTarget + followAngle);
                movementTurnPower = Range.clip(reletiveTurnAngle / Math.toRadians(30), -turnSpeed, turnSpeed);
                if (movementTurnPower >= 0 && movementTurnPower < .2)
                {
                    movementTurnPower = .2;
                }
                else if (movementTurnPower <= 0 && movementTurnPower > -.2)
                {
                    movementTurnPower = -.2;
                }
            } else {
                reletiveTurnAngle = angleWrapRad(finalAngle - GlobalHeading);
                movementTurnPower = Range.clip(reletiveTurnAngle / Math.toRadians(30), -turnSpeed, turnSpeed);
                if (movementTurnPower >= 0 && movementTurnPower < .2)
                {
                    movementTurnPower = .2;
                }
                else if (movementTurnPower <= 0 && movementTurnPower > -.2)
                {
                    movementTurnPower = -.2;
                }
            }

            //set the motors to the correct powers to move toward the target
            mecanumDrive(movementXpower, movementYpower, movementTurnPower, voltComp);
        }

        //at the end of the movement stop the motors
        drive[0].setPower(0);
        drive[1].setPower(0);
        drive[2].setPower(0);
        drive[3].setPower(0);

    }

    /**
     * this method is the key to using odometry
     * by imputing a location to drive to the robot will calculate an efficient path to the target.
     * if the robot is interfered with, it will recalculate and adjust accordingly
     *
     * at the beginning of the drive the robot will face the target (because mecanums are faster forward than sideways)
     * when it gets within a set distance of the target it will begin turning toward its desired final orientation
     *
     * this code is pulled from the basis of pure pursuit.
     * for a better understanding of pure pursuit and if someone wants to improve this code, look into learning how the rest of gluten free's code works here:
     * https://www.youtube.com/watch?v=3l7ZNJ21wMo (5 parts)
     * the code below uses the code explains in parts 1 & 2
     *
     * for the arrays; odometers should be 0.left, 1.right, 2.perpendicular
     * drive motors should be 0.Right front, 1.left front, 2.left back, 3.right back
     *
     * this version will run one calaculation and needs to be used in a loop in the parent autonomus
     */
    public void goToPosSingle(DcMotor[] odometers, DcMotor[] drive, VoltageSensor ControlHub_VoltageSensor, double x, double y, double finalAngle, double moveSpeed, double turnSpeed, double followAngle)
    {
        //bring in the encoder and motor objects
        //odometryRobotHardware robot = new odometryRobotHardware(hardwareMap);

        //update odometry location
        refresh(odometers);

        double voltComp = (14.0/ControlHub_VoltageSensor.getVoltage()) * (11.0/14.0);

        //math to calculate distances to the target
        double distanceToTarget = Math.hypot(x - GlobalX, y - GlobalY);
        double absoluteAngleToTarget = Math.atan2(x - GlobalX, y - GlobalY);
        double reletiveAngleToTarget = angleWrapRad(absoluteAngleToTarget - GlobalHeading-Math.toRadians(90));
        double reletiveXToTarget = -Math.cos(reletiveAngleToTarget) * distanceToTarget;
        double reletiveYToTarget = -Math.sin(reletiveAngleToTarget) * distanceToTarget;

        //slow down ensures the robot does not over shoot the target
        double slowDown = Range.clip(Math.log(distanceToTarget +1) / 4, .125, moveSpeed);

        //calculate the vector powers for the mecanum math
        double movementXpower = (reletiveXToTarget / (Math.abs(reletiveXToTarget) + Math.abs(reletiveYToTarget))) * slowDown;
        double movementYpower = (reletiveYToTarget / (Math.abs(reletiveYToTarget) + Math.abs(reletiveXToTarget))) * slowDown * 1.2;

        //when far away from the target the robot will point at the target to get there faster.
        //at the end of the movement the robot will begin moving toward the desired final angle
        double movementTurnPower;
        double reletiveTurnAngle;
        if (distanceToTarget > 6) {
            reletiveTurnAngle = angleWrapRad(reletiveAngleToTarget + followAngle);
            movementTurnPower = Range.clip(reletiveTurnAngle / Math.toRadians(30), -turnSpeed, turnSpeed);
            if (movementTurnPower >= 0 && movementTurnPower < .2)
            {
                movementTurnPower = .2;
            }
            else if (movementTurnPower <= 0 && movementTurnPower > -.2)
            {
                movementTurnPower = -.2;
            }
        } else {
            reletiveTurnAngle = angleWrapRad(finalAngle - GlobalHeading);
            movementTurnPower = Range.clip(reletiveTurnAngle / Math.toRadians(30), -turnSpeed, turnSpeed);
            if (movementTurnPower >= 0 && movementTurnPower < .2)
            {
                movementTurnPower = .2;
            }
            else if (movementTurnPower <= 0 && movementTurnPower > -.2)
            {
                movementTurnPower = -.2;
            }
        }

        //set the motors to the correct powers to move toward the target
        drive[0].setPower(((movementXpower - movementYpower) + (movementTurnPower) * voltComp));
        drive[1].setPower(((movementXpower + movementYpower) + (movementTurnPower) * voltComp));
        drive[2].setPower(((movementXpower - movementYpower) - (movementTurnPower) * voltComp));
        drive[3].setPower(((movementXpower + movementYpower) - (movementTurnPower) * voltComp));

    }

    public void runOpMode(){}
}
