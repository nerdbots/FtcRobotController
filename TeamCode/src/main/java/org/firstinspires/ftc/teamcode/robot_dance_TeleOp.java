/*
Copyright 2018 FIRST Tech Challenge Team [Phone] SAMSUNG

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */

public class robot_dance_TeleOp {



    private LinearOpMode opmode;

    private HardwareMap hardwareMap;

    NerdVelocityFollowing_Teleop NerdVelocityFollowing = new NerdVelocityFollowing_Teleop();


    private ElapsedTime Timer = new ElapsedTime();


    private BNO055IMU imu;
    private DcMotor frontRightMotor;
    private DcMotor rearRightMotor;
    private DcMotor frontLeftMotor;
    private DcMotor rearLeftMotor;

    private ElapsedTime elapsedTime = new ElapsedTime();
    private  ElapsedTime kickerTime = new ElapsedTime();


    private double shooterveloc = -1400;

    Orientation angles;
    Acceleration gravity;

    Orientation lastAngles = new Orientation();

    double globalAngle = 0.0;


    private ElapsedTime ZPIDTime = new ElapsedTime();

    private ElapsedTime PIDTime = new ElapsedTime();

    private ElapsedTime wobbleTime = new ElapsedTime();

    private double ZPrevError = 0;


    private double ZTotalError = 0;


    private double ZSpeed = 0;


    private double ZDerror = 0;


    private double ZkP = 0.014; //0.013
    private double ZkI = 0.000; //0.000
    private double ZkD = 0.0013;//0.0013



    private double ZTar = 0;

    private double MaxSpeedZ = 1.0;


    double prevTickTime = 0;
    int prevLeft = 0, prevRight = 0, prevLeftB = 0, prevRightB = 0;
    private final double wheelDiameter = 3.54331; // For omni wheels we are using
    private final double wheelMountAngle = 45.0; //For current drivetrain
    private final double GEAR_RATIO = 20.0 / 15.0;  // Gear ratio
    private final double ticksPerRotation = 540.0; //For omni wheels we are using
    public double [] Velocities = new double[5];
    double maxVelocity = 0.0;
    double maxAcceleration = 0.0;



    boolean sendMotorCommands = true;



    public robot_dance_TeleOp(LinearOpMode opmode) {
        this.opmode = opmode;
        this.hardwareMap = opmode.hardwareMap;
    }



    public void initializeHardware() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        rearLeftMotor = hardwareMap.get(DcMotor.class, "Rear_Left_Motor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "Front_Left_Motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "Front_Right_Motor");
        rearRightMotor = hardwareMap.get(DcMotor.class, "Rear_Right_Motor");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);


        resetAngle();

        ZPIDTime.reset();

        Timer.reset();

        Timer.reset();

        wobbleTime.reset();
    }






        double fieldCentricMororPowerFL = 0;
        double fieldCentricMororPowerFR = 0;
        double fieldCentricMororPowerRL = 0;
        double fieldCentricMororPowerRR = 0;



        double FLMP = 0;
        double FRMP = 0;
        double BLMP = 0;
        double BRMP = 0;

        double FX = 0;
        double FY = 0;

        double CA = 0;

        double RSA = 0;

        double Mag = 0;
        double zMag = 0;

        double mult = 1; //THIS IS SPEED
        double multZ = 0.6;//0.3

        double power = 1;
        double upMult = 1;

        double joyX = 0;
        double joyY = 0;





        //   resetAngle();





        // run until the end of the match (driver presses STOP)





    public void moveTeleop(double x, double y, double z) {
            //Get Joystick commands for "Smash Mouth" turning


//            //Calculation for "Smash Mouth" turning
//            zMag = (joyX * joyX) + (joyY * joyY);
//
//
//            if (Math.sqrt(zMag) > 0.5)
//                ZTar = Math.atan2(-joyX, -joyY) * 180 / 3.14159;


            FX = -x;
            FY = -y;


            //Direct tank turning control if slowmode is on

//        PIDArm(getAngle(), z, ZkP, ZkI, ZkD, 67);//CAN BE ANYTHING BUT 0 OR 1


//        LMP = (ZSpeed * multZ) + FX; //multZ will only affect Z. This is because if joypad Z is zero then Z is zero.
//        RMP = (ZSpeed * multZ) - FX;
//        FMP = (ZSpeed * multZ) + FY;
//        BMP = (ZSpeed * multZ) - FY;

        FLMP = -x+y+z;
        FRMP = -x-y+z;
        BLMP =  x+y+z;
        BRMP =  x-y+z;


            //Outputs field-centric motor commands so Velocity Following
            fieldCentricMororPowerFL = FLMP * mult;
            fieldCentricMororPowerRR = BRMP * mult;

            fieldCentricMororPowerRL = BLMP * mult;
            fieldCentricMororPowerFR = FRMP * mult;

            VelocityCommands(fieldCentricMororPowerFL, fieldCentricMororPowerRR, fieldCentricMororPowerRL, fieldCentricMororPowerFR);


        }


    //0 is rearMotor 1 is frontMotor \/
    private void resetAngle () {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    public void PIDArm ( double EV, double TPos, double kP, double kI, double kD, int motor){

        double DError = 0;
        int DBanMin = -1;
        int DBanMax = 1;
        int MaxError = 10;
        double error = 0;
        double speed = 0;
        double TotalError = 0;
        double PrevError = 0;
        double MaxSpeed = 0;


        TotalError = ZTotalError;
        PrevError = ZPrevError;
        PIDTime = ZPIDTime;
        MaxSpeed = MaxSpeedZ;




        //calculate error (Proportional)
        error = TPos - EV;

        //Calculate Total error (Integral)
        TotalError = (error * PIDTime.seconds()) + TotalError;

        //do deadban
        if (DBanMax > error && error > DBanMin) {
            error = 0;
            //TotalError = 0;
        }

        //calculate delta error (Derivative)
        DError = -(EV/*error*/ - PrevError) / PIDTime.seconds();

        //reset elapsed timer
        PIDTime.reset();

        //Max total error
        if (Math.abs(TotalError) > MaxError) {


            if (TotalError > 0) {
                TotalError = MaxError;
            } else {
                TotalError = -MaxError;
            }

        }


        //Calculate final speed
        speed = (error * kP) + (TotalError * kI) + (DError * kD);


        //Make sure speed is no larger than MaxSpeed
        if (Math.abs(speed) > MaxSpeed) {
            if (speed > 0) {
                speed = MaxSpeed;
            } else {
                speed = -MaxSpeed;
            }
        }
        PrevError = EV/*error*/;




    }


    //Function to get the angle of the Gyro sensor
    private double getAngle () {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle;
        deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    private void VelocityCommands(double frontLeftMotorPower, double rearRightMotorPower, double rearLeftMotorPower, double frontRightMotorPower) {

        double frontLeftMotorTarget = powerToSpeed(frontLeftMotorPower);
        double rearRightMotorTarget = powerToSpeed(rearRightMotorPower);
        double frontRightMotorTarget = powerToSpeed(frontRightMotorPower);
        double rearLeftMotorTarget = powerToSpeed(rearLeftMotorPower);

        getVelocityForCurrentLoop();

        double frontLeftMotorSpeed = Velocities[0];
        double rearRightMotorSpeed = Velocities[3];
        double frontRightMotorSpeed = Velocities[1];
        double rearLeftMotorSpeed = Velocities[2];
        double deltaTime = Velocities[4];

        double [] motorSpeedCommand = NerdVelocityFollowing.velocityFollowing(frontLeftMotorTarget, rearRightMotorTarget,
                frontRightMotorTarget, rearLeftMotorTarget, frontLeftMotorSpeed, rearRightMotorSpeed, frontRightMotorSpeed,
                rearLeftMotorSpeed, deltaTime);


        if(sendMotorCommands) {
            frontLeftMotor.setPower(motorSpeedCommand[0]);
            rearRightMotor.setPower(motorSpeedCommand[3]);
            frontRightMotor.setPower(motorSpeedCommand[1]);
            rearLeftMotor.setPower(motorSpeedCommand[2]);
        }



//        telemetry.addData("flCommandedVelocity", frontLeftMotorTarget);
//        telemetry.addData("frCommandedVelocity", frontRightMotorTarget);
//        telemetry.addData("rlCommandedVelocity", rearLeftMotorTarget);
//        telemetry.addData("rrCommandedVelocity", rearRightMotorTarget);
//        telemetry.addData("flActualVelocity", Velocities[0]);
//        telemetry.addData("frActualVelocity", Velocities[1]);
//        telemetry.addData("rlActualVelocity", Velocities[2]);
//        telemetry.addData("rrActualVelocity", Velocities[3]);
//        telemetry.addData("average lag (IPS)", (((Velocities[0]+Velocities[1]+Velocities[2]+Velocities[3])/4)-((frontLeftMotorTarget+frontRightMotorTarget+rearLeftMotorTarget+rearRightMotorTarget)/4)));
//
//
//        telemetry.addData("Status", "Running");
//        telemetry.update();

    }


    public double [] getVelocityForCurrentLoop() {
        int leftCurrent = frontLeftMotor.getCurrentPosition();
        int rightCurrent = frontRightMotor.getCurrentPosition();
        int leftBCurrent = rearLeftMotor.getCurrentPosition();
        int rightBCurrent = rearRightMotor.getCurrentPosition();

        double tTime = elapsedTime.seconds();
        double deltaTickTime = tTime - prevTickTime;
        prevTickTime = tTime;

        double leftInches = ticksToInches((int) (leftCurrent - prevLeft), wheelDiameter, wheelMountAngle);
        double rightInches = ticksToInches((int) (rightCurrent - prevRight), wheelDiameter, wheelMountAngle);
        double leftBInches = ticksToInches((int) (leftBCurrent - prevLeftB), wheelDiameter, wheelMountAngle);
        double rightBInches = ticksToInches((int) (rightBCurrent - prevRightB), wheelDiameter, wheelMountAngle);
        double frontLeftVelocity = leftInches / deltaTickTime;
        double frontRightVelocity = rightInches / deltaTickTime;
        double rearLeftVelocity = leftBInches / deltaTickTime;
        double rearRightVelocity = rightBInches / deltaTickTime;
        double avgVelocity = (frontLeftVelocity + frontRightVelocity + rearLeftVelocity + rearRightVelocity) / 4;
        double currentAcceleration = avgVelocity / deltaTickTime;
        if (currentAcceleration > maxAcceleration) {
            maxAcceleration = currentAcceleration;
        }
        if (avgVelocity > maxVelocity) {
            maxVelocity = avgVelocity;
        }
        int leftDeltaTicks = leftCurrent - prevLeft;
        int rightDeltaTicks = rightCurrent - prevRight;
        int leftBDeltaTicks = leftBCurrent - prevLeftB;
        int rightBDeltaTicks = rightBCurrent - prevRightB;
        double avgDeltaTicks = (leftDeltaTicks + rightDeltaTicks + leftBDeltaTicks + rightBDeltaTicks) / 4;
        prevLeft = leftCurrent;
        prevRight = rightCurrent;
        prevLeftB = leftBCurrent;
        prevRightB = rightBCurrent;
        Velocities[0] = frontLeftVelocity;
        Velocities[1] = frontRightVelocity;
        Velocities[2] = rearLeftVelocity;
        Velocities[3] = rearRightVelocity;
        Velocities[4] = deltaTickTime; // added delta time to be used in PID

        return Velocities;  //inches per second
    }

    public double ticksToInches(int ticks, double wheelDiameter, double wheelMountAngle) {
        double circum = wheelDiameter * Math.PI;
        double numberofWheelRotations = (double) ticks / ticksPerRotation;
        double wheelDistanceToTravel = numberofWheelRotations * circum;
//        double straightDistanceToTravel = wheelDistanceToTravel; // (Math.cos(Math.toRadians(wheelMountAngle)) * GEAR_RATIO);
        return wheelDistanceToTravel;
    }

    private double powerToSpeed (double motorPower){
        double wheelSpeedRPS = motorPower * 5200 / 60 / 19.2; //convert motor power to wheel rotations per second, 6000 rpm max motor speed, 60 seconds in a minute.
        double wheelSpeedIPS = wheelSpeedRPS * wheelDiameter * Math.PI; //
        return wheelSpeedIPS;

    }




}

