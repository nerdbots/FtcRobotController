package treamcode;/*
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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import treamcode.NerdHardwareWorld;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Timer;


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
public class NerdPIDCalc_MotionProfiling_Clean {
    private LinearOpMode opmode;
    private HardwareMap hardwareMap;
    private ElapsedTime moveTime = new ElapsedTime();

    NerdHardwareWorld HWorld;

    double angles;
    Acceleration gravity;

    Orientation lastAngles = new Orientation();

    double globalAngle = 0.0;

    private BNO055IMU imu = null;   // Gyro device


    public DcMotor leftMotor;
    public DcMotor rightMotor;
    public DcMotor leftMotorB;
    public DcMotor rightMotorB;
    private ElapsedTime PIDTime = new ElapsedTime();
    private double FLPrevError = 0;
    private double FLTotalError = 0;
    public double FLVeloc = 0;
    private double FLMaxVeloc = 10;
    private double FLP = 0;
    private double FLI = 0;
    private double FLD = 0;
    private double FLkP;
    private double FLkI;
    private double FLkD;
    private ElapsedTime FLPIDTime = new ElapsedTime();
    private double FRPrevError = 0;
    private double FRTotalError = 0;
    public double FRVeloc = 0;
    private double FRMaxVeloc = 10;
    private double FRP = 0;
    private double FRI = 0;
    private double FRD = 0;
    private double FRkP;
    private double FRkI;
    private double FRkD;
    private ElapsedTime FRPIDTime = new ElapsedTime();
    private double RLPrevError = 0;
    private double RLTotalError = 0;
    public double RLVeloc = 0;
    private double RLMaxVeloc = 10;
    private double RLP = 0;
    private double RLI = 0;
    private double RLD = 0;
    private double RLkP;
    private double RLkI;
    private double RLkD;
    private ElapsedTime RLPIDTime = new ElapsedTime();
    private double RRPrevError = 0;
    private double RRTotalError = 0;
    public double RRVeloc = 0;
    private double RRMaxVeloc = 10;
    private double RRP = 0;
    private double RRI = 0;
    private double RRD = 0;
    private double RRkP;
    private double RRkI;
    private double RRkD;
    private ElapsedTime RRPIDTime = new ElapsedTime();

    private double gyroPrevError = 0;
    private double gyroTotalError = 0;
    public double gyroVeloc = 0;
    private double gyroMaxVeloc = 10;
    private double gyroP = 0;
    private double gyroI = 0;
    private double gyroD = 0;
    private double gyrokP;
    private double gyrokI;
    private double gyrokD;
    private ElapsedTime gyroPIDTime = new ElapsedTime();


    public ElapsedTime driveTime = new ElapsedTime();

    public ElapsedTime tickTime = new ElapsedTime();

    public double[] Velocities = new double[4];

    private double deltaTickTime = 0;

    private double prevTickTime = 0;

    private double tTime = 0;

    public boolean debugFlag = false;
    public boolean debugFlag2 = false;

    public int ACDStage = 0;

    private double prevDriveTime;

    public int LWM = -1;

    public int RWM = 1;





    ArrayList<Double> rAverage = new ArrayList<>(Arrays.asList(0.0,0.0,0.0,0.0));
    ArrayList<Double> FLrAverage = new ArrayList<>(Arrays.asList(0.0,0.0,0.0,0.0));
    ArrayList<Double> FRrAverage = new ArrayList<>(Arrays.asList(0.0,0.0,0.0,0.0));
    ArrayList<Double> RLrAverage = new ArrayList<>(Arrays.asList(0.0,0.0,0.0,0.0));
    ArrayList<Double> RRrAverage = new ArrayList<>(Arrays.asList(0.0,0.0,0.0,0.0));

    double FLInVeloc = 0;
    double FRInVeloc = 0;
    double RLInVeloc = 0;
    double RRInVeloc = 0;

    private double sum = 0;

    public double rAverageSize = 6;

    public double Kramp;

    ElapsedTime runtime = new ElapsedTime();
    double prevRuntime = 0.0;
    double frontLeftVelocity = 0, frontRightVelocity = 0, rearLeftVelocity = 0, rearRightVelocity = 0;
    int leftCurrent, rightCurrent, rightBCurrent, leftBCurrent;
    int prevLeft = 0, prevRight = 0, prevLeftB = 0, prevRightB = 0;
    double leftInches, rightInches, leftBInches, rightBInches;
    double currentAcceleration = 0.0;
    double avgVelocity = 0.0;
    double maxVelocity = 0.0;
    double maxAcceleration = 0.0;
    //int prevLeftTicks = 0, prevRightTicks = 0, prevLeftBTicks = 0, prevRightBTicks = 0;
    int leftDeltaTicks = 0, rightDeltaTicks = 0, leftBDeltaTicks = 0, rightBDeltaTicks = 0, avgDeltaTicks;
    //double[] Velocities = {frontLeftVelocity, frontRightVelocity, rearLeftVelocity, rearRightVelocity};
    double loopTime = 0.05;
    private final double wheelDiameter = 3.54331; // For omni wheels we are using
    private final double wheelMountAngle = 45.0; //For current drivetrain
    private final double GEAR_RATIO = 20.0 / 15.0;  // Gear ratio
    private final double ticksPerRotation = 560.0; //For omni wheels we are using
    public Timer timer = new Timer();
    ElapsedTime maxVelocTime = new ElapsedTime();

    public double maxPower = 0;

    public NerdPIDCalc_MotionProfiling_Clean(LinearOpMode opmode) {
        this.opmode = opmode;
        this.hardwareMap = opmode.hardwareMap;
    }

    public void setFLGains(double kPV, double kIV, double kDV) {
        FLkP = kPV;
        FLkI = kIV;
        FLkD = kDV;
    }

    public void setFRGains(double kPV, double kIV, double kDV) {
        FRkP = kPV;
        FRkI = kIV;
        FRkD = kDV;
    }

    public void setRLGains(double kPV, double kIV, double kDV) {
        RLkP = kPV;
        RLkI = kIV;
        RLkD = kDV;
    }

    public void setRRGains(double kPV, double kIV, double kDV) {
        RRkP = kPV;
        RRkI = kIV;
        RRkD = kDV;
    }

    public void setGyroGains(double kPV, double kIV, double kDV) {
        gyrokP = kPV;
        gyrokI = kIV;
        gyrokD = kDV;
        moveTime.reset();
    }

    public void resetFL() {
        FLPrevError = 0;
        FLTotalError = 0;
        FLVeloc = 0;
        FLPIDTime.reset();
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetFR() {
        FRPrevError = 0;
        FRTotalError = 0;
        FRVeloc = 0;
        FRPIDTime.reset();
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetRL() {
        RLPrevError = 0;
        RLTotalError = 0;
        RLVeloc = 0;
        RLPIDTime.reset();
        leftMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetRR() {
        RRPrevError = 0;
        RRTotalError = 0;
        RRVeloc = 0;
        RRPIDTime.reset();
        rightMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void resetgyroPID_angle() {
        gyroPrevError = 0;
        gyroTotalError = 0;
        gyroVeloc = 0;
        gyroPIDTime.reset();

    }

    public void PIDVeloc(double CurrVeloc, double TVeloc, String Motor) {
        double DError = 0;
        int DBanMin = -1;
        int DBanMax = 1;
        double error = 0;
        double speed = 0;
        double TotalError = 0;
        double PrevError = 0;
        double MaxVeloc = 0;
        double kP = 0, kI = 0, kD = 0;
        switch (Motor) {
            case "FL":
                TotalError = FLTotalError;
                PrevError = FLPrevError;
                MaxVeloc = FLMaxVeloc;
                PIDTime = FLPIDTime;
                kP = FLkP;
                kI = FLkI;
                kD = FLkD;
                break;
            case "FR":
                TotalError = FRTotalError;
                PrevError = FRPrevError;
                MaxVeloc = FRMaxVeloc;
                PIDTime = FRPIDTime;
                kP = FRkP;
                kI = FRkI;
                kD = FRkD;
                break;
            case "RL":
                TotalError = RLTotalError;
                PrevError = RLPrevError;
                MaxVeloc = RLMaxVeloc;
                PIDTime = RLPIDTime;
                kP = RLkP;
                kI = RLkI;
                kD = RLkD;
                break;
            case "RR":
                TotalError = RRTotalError;
                PrevError = RRPrevError;
                MaxVeloc = RRMaxVeloc;
                PIDTime = RRPIDTime;
                kP = RRkP;
                kI = RRkI;
                kD = RRkD;
                break;

            case "gyro":
                TotalError = gyroTotalError;
                PrevError = gyroPrevError;
                MaxVeloc = gyroMaxVeloc;
                PIDTime = gyroPIDTime;
                kP = gyrokP;
                kI = gyrokI;
                kD = gyrokD;
                break;
        }

        //calculate error (Proportional)
        error = TVeloc - CurrVeloc;
        //Calculate Total error (Integral)
        TotalError = (error * PIDTime.seconds()) + TotalError;
        PIDTime.reset();
        //do deadban
        if (DBanMax > error && error > DBanMin) {
            error = 0;
            //TotalError = 0;
        }
        //calculate delta error (Derivative)
        DError = (error - PrevError) / PIDTime.seconds();
        speed = (error * kP) + (TotalError * kI) + (DError * kD);


        PrevError = error;



        switch (Motor) {
            case "FL":
                FLTotalError = TotalError;
                FLPrevError = PrevError;
                FLVeloc = speed;
                FLP = error;
                FLI = TotalError;
                FLD = DError;
                opmode.telemetry.addData("errorFL", error);
                opmode.telemetry.addData("totalErrorFL", TotalError);

            case "FR":
                FRTotalError = TotalError;
                FRPrevError = PrevError;
                FRVeloc = speed;
                FRP = error;
                FRI = TotalError;
                FRD = DError;
                opmode.telemetry.addData("errorFR", error);
                opmode.telemetry.addData("totalErrorFL", TotalError);

                break;
            case "RL":
                RLTotalError = TotalError;
                RLPrevError = PrevError;
                RLVeloc = speed;
                RLP = error;
                RLI = TotalError;
                RLD = DError;
                opmode.telemetry.addData("errorRL", error);
                opmode.telemetry.addData("totalErrorFL", TotalError);

                break;
            case "RR":
                RRTotalError = TotalError;
                RRPrevError = PrevError;
                RRVeloc = speed;
                RRP = error;
                RRI = TotalError;
                RRD = DError;
                opmode.telemetry.addData("errorRR", error);
                opmode.telemetry.addData("totalErrorFL", TotalError);

                break;

            case "gyro":
                gyroTotalError = TotalError;
                gyroPrevError = PrevError;
                gyroVeloc = speed;
                gyroP = error;
                gyroI = TotalError;
                gyroD = DError;
                opmode.telemetry.addData("error gyro", error);
                opmode.telemetry.addData("totalError gyro", TotalError);
                break;

        }

        if(Math.abs(FLVeloc) > Math.abs(FRVeloc)){
            maxPower = Math.abs(FLVeloc);
        } else {
            maxPower = Math.abs(FRVeloc);
        }
        if(Math.abs(RLVeloc) > maxPower){
            maxPower = Math.abs(RLVeloc);
        }
        if(Math.abs(RRVeloc) > maxPower) {
            maxPower = Math.abs(RRVeloc);
        }

        if(maxPower > 1) {
            FLVeloc /= maxPower;
            FRVeloc /= maxPower;
            RLVeloc /= maxPower;
            RRVeloc /= maxPower;
        }

    }

    public double ticksToInches(int ticks, double wheelDiameter, double wheelMountAngle) {
        double circum = wheelDiameter * 3.14159265;
        double numberofWheelRotations = (double) ticks / ticksPerRotation;
        double wheelDistanceToTravel = numberofWheelRotations * circum;
        double straightDistanceToTravel = wheelDistanceToTravel / (Math.cos(Math.toRadians(wheelMountAngle)) * GEAR_RATIO);
        return straightDistanceToTravel;
    }

    public double[] getVelocityForCurrentLoop() {
        leftCurrent = leftMotor.getCurrentPosition();
        rightCurrent = rightMotor.getCurrentPosition();
        leftBCurrent = leftMotorB.getCurrentPosition();
        rightBCurrent = rightMotorB.getCurrentPosition();

        tTime = tickTime.seconds();

        deltaTickTime = tTime - prevTickTime;

        leftInches = ticksToInches((int) (leftCurrent - prevLeft), wheelDiameter, wheelMountAngle);
        rightInches = ticksToInches((int) (rightCurrent - prevRight), wheelDiameter, wheelMountAngle);
        leftBInches = ticksToInches((int) (leftBCurrent - prevLeftB), wheelDiameter, wheelMountAngle);
        rightBInches = ticksToInches((int) (rightBCurrent - prevRightB), wheelDiameter, wheelMountAngle);
        frontLeftVelocity = leftInches / deltaTickTime;
        frontRightVelocity = rightInches / deltaTickTime;
        rearLeftVelocity = leftBInches / deltaTickTime;
        rearRightVelocity = rightBInches / deltaTickTime;
        avgVelocity = (frontLeftVelocity + frontRightVelocity + rearLeftVelocity + rearRightVelocity) / 4;
        currentAcceleration = avgVelocity / deltaTickTime;
        if (currentAcceleration > maxAcceleration) {
            maxAcceleration = currentAcceleration;
        }
        if (avgVelocity > maxVelocity) {
            maxVelocity = avgVelocity;
        }
        leftDeltaTicks = leftCurrent - prevLeft;
        rightDeltaTicks = rightCurrent - prevRight;
        leftBDeltaTicks = leftBCurrent - prevLeftB;
        rightBDeltaTicks = rightBCurrent - prevRightB;
        avgDeltaTicks = (leftDeltaTicks + rightDeltaTicks + leftBDeltaTicks + rightBDeltaTicks) / 4;
        prevLeft = leftCurrent;
        prevRight = rightCurrent;
        prevLeftB = leftBCurrent;
        prevRightB = rightBCurrent;
        Velocities[0] = frontLeftVelocity;
        Velocities[1] = frontRightVelocity;
        Velocities[2] = rearLeftVelocity;
        Velocities[3] = rearRightVelocity;

        prevTickTime = tTime;

        return Velocities;
    }


    public void runMotors(double FLinTar, double FRinTar, double RLinTar, double RRinTar, double KV, double accelRate) {


//todo: if run once, uncomment

//        tickTime.reset();



        //   opmode.sleep(1000);
        if (this.opmode.opModeIsActive()) {
            opmode.telemetry.addLine("Inside opmodeisactive");
            // opmode.telemetry.update();


            if(debugFlag2){
                RobotLog.d("Runtime, Target Veloc, FL Veloc, FR Veloc, RL Veloc, RR Veloc, FLP, FRP, RLP, RRP");
            }








            opmode.telemetry.addData("stage", ACDStage);

            opmode.telemetry.addLine("run started");


            opmode.telemetry.addLine("runtime reset");


            getVelocityForCurrentLoop();


            runtime.reset();
            opmode.telemetry.addLine("Velocity gotten");


            FLInVeloc = Velocities[0];
            FRInVeloc = Velocities[1];
            RLInVeloc = Velocities[2];
            RRInVeloc = Velocities[3];

            PIDVeloc(FLrunningAverage(FLInVeloc), LWM * rampFL(KV, accelRate, FLinTar, FLInVeloc), "FL");
            PIDVeloc(FRrunningAverage(FRInVeloc), RWM * rampFR(KV, accelRate, FRinTar, FRInVeloc), "FR");
            PIDVeloc(RLrunningAverage(RLInVeloc), LWM * rampRL(KV, accelRate, RLinTar, RLInVeloc), "RL");
            PIDVeloc(RRrunningAverage(RRInVeloc), RWM * rampRR(KV, accelRate, RRinTar, RRInVeloc), "RR");



            opmode.telemetry.addLine("PID done");
            //  opmode.telemetry.update();
            //      opmode.sleep(1000);

            leftMotor.setPower(FLVeloc-gyroVeloc);
            rightMotor.setPower(FRVeloc+gyroVeloc);
            leftMotorB.setPower(RLVeloc-gyroVeloc);
            rightMotorB.setPower(RRVeloc+gyroVeloc);

            opmode.telemetry.addData("FLPower", FLVeloc);
            opmode.telemetry.addData("FRPower", FRVeloc);
            opmode.telemetry.addData("RLPower", RLVeloc);
            opmode.telemetry.addData("RRPower", RRVeloc);

            opmode.telemetry.addLine("power set");

            opmode.telemetry.addLine("looptime gotten");


            // opmode.telemetry.addData("Motor speeeeeedd", FLVeloc);
            opmode.telemetry.addData("VelocityFL", Velocities[0]);
            opmode.telemetry.addData("VelocityFR", Velocities[1]);
            opmode.telemetry.addData("VelocityRL", Velocities[2]);
            opmode.telemetry.addData("VelocityRR", Velocities[3]);
            opmode.telemetry.addData("Average motor POWAAAH", ((leftMotor.getPower() + leftMotorB.getPower() + rightMotor.getPower() +
                    rightMotorB.getPower()) / 4));
            opmode.telemetry.update();



        }
        opmode.telemetry.addLine("run ended");
        //opmode.telemetry.update();
        //  opmode.sleep(1000);
    }

    public void initializeHardware() {

        HWorld = new NerdHardwareWorld();

        this.leftMotor = HWorld.FLM;
        this.rightMotor = HWorld.FRM;
        this.leftMotorB = HWorld.RLM;
        this.rightMotorB = HWorld.RRM;

        this.imu = HWorld.imu;

        resetAngle();

    }

    public double FLrunningAverage(double inputValueFL) {


        FLrAverage.add(inputValueFL);

        if (rAverage.size() > rAverageSize)
            FLrAverage.remove(0);

        sum = 0;
        for(int i = 0; i < rAverage.size(); i++)
            sum += FLrAverage.get(i);

        opmode.telemetry.addData("sumFL", sum);


        //return inputValue;
        return sum/FLrAverage.size();


    }
    public double FRrunningAverage(double inputValueFR) {


        FRrAverage.add(inputValueFR);

        if (rAverage.size() > rAverageSize)
            FRrAverage.remove(0);

        sum = 0;
        for(int i = 0; i < rAverage.size(); i++)
            sum += FRrAverage.get(i);

        opmode.telemetry.addData("sumFR", sum);


        //return inputValue;
        return sum/FRrAverage.size();


    }
    public double RLrunningAverage(double inputValueRL) {


        RLrAverage.add(inputValueRL);

        if (rAverage.size() > rAverageSize)
            RLrAverage.remove(0);

        sum = 0;
        for(int i = 0; i < rAverage.size(); i++)
            sum += RLrAverage.get(i);

        opmode.telemetry.addData("sumRL", sum);


        //return inputValue;
        return sum/RLrAverage.size();


    }
    public double RRrunningAverage(double inputValueRR) {


        RRrAverage.add(inputValueRR);

        if (rAverage.size() > rAverageSize)
            RRrAverage.remove(0);

        sum = 0;
        for(int i = 0; i < rAverage.size(); i++)
            sum += RRrAverage.get(i);

        opmode.telemetry.addData("sumRR", sum);


        //return inputValue;
        return sum/RRrAverage.size();


    }

    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }
    public  double getAngle(){

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    public int rampFL(double KVeloc, double accelRate, double tarVeloc, double currVeloc) {

        return (int) Math.round(KVeloc*tarVeloc-(accelRate*((KVeloc*tarVeloc)-currVeloc)));

    }public int rampFR(double KVeloc, double accelRate, double tarVeloc, double currVeloc) {

        return (int) Math.round(KVeloc*tarVeloc-(accelRate*((KVeloc*tarVeloc)-currVeloc)));

    }public int rampRL(double KVeloc, double accelRate, double tarVeloc, double currVeloc) {

        return (int) Math.round(KVeloc*tarVeloc-(accelRate*((KVeloc*tarVeloc)-currVeloc)));

    }public int rampRR(double KVeloc, double accelRate, double tarVeloc, double currVeloc) {

        return (int) Math.round(KVeloc*tarVeloc-(accelRate*((KVeloc*tarVeloc)-currVeloc)));

    }

    //Usually put right after WaitForStart();
    public void resetAll() {
        runtime.reset();

        maxVelocTime.reset();
        resetFL();
        resetFR();
        resetRL();
        resetRR();
        resetgyroPID_angle();
        tickTime.reset();
        driveTime.reset();

        LWM = -1;
        RWM = 1;

    }
}
