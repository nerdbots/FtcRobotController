package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import company.ComputerDebugging;
import company.FloatPoint;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import opencv.core.PointPP;
import treamcode.CurvePoint;
import treamcode.MathFunctions;
import treamcode.NerdPID_PurePursuit;
import treamcode.NerdVelocityFollowing;

import java.util.ArrayList;

public class PurePursuitRobotMovement6 {

    private boolean debugFlag=false;

    //We need an opmode to get the hardware map etc.

    private LinearOpMode opmode;

    private HardwareMap hardwareMap;

    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor rearLeftMotor;
    private DcMotor rearRightMotor;

    private DcMotorEx frontEncoder;
    private DcMotor rightEncoder;
    private DcMotor leftEncoder;
    private DcMotor backEncoder;

    Servo wobbleServo;
    private Servo indexingServo;

    public static final double indexerHomePos = 1.0;
    public static final double indexerPushedPos = 0.45;

//    private DcMotor wobbleMotor;
//    Servo wobbleServo;

//    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime elapsedTime = new ElapsedTime();
    private ElapsedTime Timer = new ElapsedTime();

    private BNO055IMU imu = null;   // Gyro device

    Orientation angles;
    Acceleration gravity;

    Orientation lastAngles = new Orientation();

    private double robotAngleToField = 0;
    private double robotAngleToTarget = 0;
    private double motorAngleToTarget = 0;
    private double zPIDAngle = 0;

    private double xPosition = 0;
    private double yPosition = 0;
    private double displacementX = 0;
    private double displacementY = 0;

    double xPositionOpt = 0;
    double yPositionOpt = 0;


    private double xPositionOpticalInit = 0;
    private double yPositionOpticalInit = 0;
    private double displacementXOptical = 0;
    private double displacementYOptical = 0;

    double frontDispNoRotTotOpt = 0;
    double leftDispNoRotTotOpt = 0;
    double rightDispNoRotTotOpt = 0;
    double rearDispNoRotTotOpt = 0;

    double frontPositionOptical = 0;
    double rightPositionOptical = 0;
    double leftPositionOptical = 0;
    double backPositionOptical = 0;
    double [] robotPositionXYOptical;

    double omniDriveFactorOpt = 0;

    double frontDisplacementOld = 0;
    double leftDisplacementOld = 0;
    double rightDisplacementOld = 0;
    double rearDisplacementOld = 0;

    double robotRot = 0;
    double robotRotNew = 0;
    double robotRotOld = 0;
    double robotRotDisplacement = 0;
    double robotVectorByOdo = 0;
    double rfDisplacement = 0;
    double rfDisplacementNew = 0;
    double rfDisplacementOld = 0;
    double lfDisplacement = 0;
    double lfDisplacementNew = 0;
    double lfDisplacementOld = 0;
    double rrDisplacement = 0;
    double rrDisplacementNew = 0;
    double rrDisplacementOld = 0;
    double lrDisplacement = 0;
    double lrDisplacementNew = 0;
    double lrDisplacementOld = 0;
    double rfDispNoRot = 0;
    double lfDispNoRot = 0;
    double rrDispNoRot = 0;
    double lrDispNoRot = 0;
    double rfDispNoRotTot = 0;
    double lfDispNoRotTot = 0;
    double rrDispNoRotTot = 0;
    double lrDispNoRotTot = 0;
    double robotVectorMag = 0;
    double omniDriveAngle = 0;
    double omniDriveFactor = 0;


    double robotRotNewOpt = 0;
    double robotRotOldOpt = 0;
    double robotRotOpt = 0;
    //double robotRotDisplacementOptF = 0;
    double robotXdisplacementOpt = 0;
    double robotYdisplacementOpt = 0;
    double robotVectorByOdoOpt = 0;
    double robotVectorMagOpt = 0;
    double robotFieldAngleOpt = 0;
    double robotXdisplacementOptTot = 0;
    double robotYdisplacementOptTot = 0;


    double robotFieldPositionX = 0;
    double robotFieldPositionY = 0;
    double robotFieldAngle = 0;
    double [] robotPositionXY;
    double robotXdisplacement = 0;
    double robotYdisplacement = 0;

    double distanceToTarget = 10;
    double prevDistanceToTarget = 20;
    double distanceToTargetAngle = 0;
    double distanceToEndPoint = 10;

//    static final double DISTANCE_THRESHOLD = 2;

    double deltaTime = 0.024;
    double startTime = 0;
    double oldTime = 0;
    double loopTime = 0;
    double currentTime = 0;
    double parkTimer = 0;


    double xPower = 0;
    double yPower = 0;
    double zPower = 0;
    double zPowerStart = -0.20;
    double zPowerIncrease = 0.075;

    boolean useZPID = false;

    double zSpeedTargetAngle = 0;

    double robotTargetSpeed = 0.5;
    double robotTargetAngle = 0;
    double robotTurnSpeed = 0;

    double robotSpeed = 0;
    double decelRate =0;
    double robotTargetSpeedIPS = 0;
    double robotTargetSpeedPID = 0;

    double frontLeftMotorPower = 0;
    double frontRightMotorPower = 0;
    double rearLeftMotorPower = 0;
    double rearRightMotorPower = 0;
    double maxPowerEndPP = 0;

    //for Jusnoor's code
    double prevTickTime = 0;
    int prevLeft = 0, prevRight = 0, prevLeftB = 0, prevRightB = 0;
    private final double wheelDiameter = 3.54331; // For omni wheels we are using
    private final double wheelMountAngle = 45.0; //For current drivetrain
    private final double GEAR_RATIO = 20.0 / 15.0;  // Gear ratio
    private final double ticksPerRotation = 540.0; //For omni wheels we are using
    public double [] Velocities = new double[5];
    double maxVelocity = 0.0;
    double maxAcceleration = 0.0;



    double shooterVeloc = 1000;


    /**
     * Constructor to create NerdBOT object
     * <p>
     * Creates a new NerdBOT object and assigns the hardwareMap provided by caller
     *
     * @param opmode Hardware Map provided by the calling OpMode.
     *               NerdBOT takes an opmode object so that it can get the hardwareMap.     *
     */

    public PurePursuitRobotMovement6(LinearOpMode opmode) {
        this.opmode = opmode;
        this.hardwareMap = opmode.hardwareMap;
    }

    //Function to initialize hardware components.

    public void setDebug(boolean debugFlag){
        this.debugFlag=debugFlag;
    }


    public void initializeHardware(){

        //Initialize Motors

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        this.frontLeftMotor = this.hardwareMap.get(DcMotor.class, "Front_Left_Motor");
        this.frontRightMotor = this.hardwareMap.get(DcMotor.class, "Front_Right_Motor");
        this.rearLeftMotor = this.hardwareMap.get(DcMotor.class, "Rear_Left_Motor");
        this.rearRightMotor = this.hardwareMap.get(DcMotor.class, "Rear_Right_Motor");

        this.frontEncoder = this.hardwareMap.get(DcMotorEx.class, "Front");
        this.rightEncoder = this.hardwareMap.get(DcMotor.class, "Right");
        this.leftEncoder = this.hardwareMap.get(DcMotor.class, "Left");
        this.backEncoder = this.hardwareMap.get(DcMotor.class, "Back");

        this.wobbleServo = hardwareMap.get(Servo.class, "wobble_Goal_Servo");
        this.indexingServo = hardwareMap.get(Servo.class, "indexingServo");

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".

        this.imu = this.hardwareMap.get(BNO055IMU.class, "imu");
        this.imu.initialize(parameters);
        this.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        resetAngle();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        this.frontEncoder.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        this.rightEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.backEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.frontEncoder.setVelocityPIDFCoefficients(200, 0.1, 0, 16);

        this.frontEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.backEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //this.frontEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        this.rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.backEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //this.frontEncoder.setDirection(DcMotorEx.Direction.REVERSE);
        this.rightEncoder.setDirection(DcMotor.Direction.REVERSE);
        this.leftEncoder.setDirection(DcMotor.Direction.REVERSE);
        this.backEncoder.setDirection(DcMotor.Direction.REVERSE);

        xPosition = 0;
        yPosition = 0;

        displacementX = 0;
        displacementY = 0;

        this.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.rearLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.rearRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




        xPositionOpticalInit = 0;
        yPositionOpticalInit = 0;

        xPositionOpt = 0;
        yPositionOpt = 0;

        displacementXOptical = 0;
        displacementYOptical = 0;

        prevDistanceToTarget = 20;

        //Initializes motors (obvi)
//        this.wobbleMotor = hardwareMap.get(DcMotor.class, "Left");
//        this.wobbleServo = hardwareMap.get(Servo.class, "wobble_Goal_Servo");


    }


    private void resetAngle() {


        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        robotAngleToField = 0;
    }

    //Function to get the angle of the Gyro sensor
    private double getAngle() {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        robotAngleToField += deltaAngle;

        lastAngles = angles;

        return robotAngleToField;
    }

    public void goToPosition(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed, double parkDistance, double parkRadius){
//        startTime = runtime;

        distanceToTarget = 10;
        distanceToTargetAngle = getAngle() + 90;

//        double [] robotPositionXYOpticalInit = findDisplacementOptical();
//        xPositionOpticalInit = robotPositionXYOpticalInit[4];
//        yPositionOpticalInit = robotPositionXYOpticalInit[5];

        while (this.opmode.opModeIsActive() && !this.opmode.isStopRequested() && !distanceTargetReached(distanceToTarget, parkRadius)) {

            //First calculate motor speeds for linear (x, y) motion

            currentTime = elapsedTime.seconds();
            loopTime = currentTime - oldTime;
            oldTime = currentTime;
            deltaTime = currentTime - startTime;

            robotPositionXY = findDisplacement(xPosition, yPosition, robotVectorByOdo);

            robotPositionXYOptical = findDisplacementOptical();

            //distanceToTarget = Math.hypot(x - robotPositionXY[0], y - robotPositionXY[1]);
            distanceToTarget = Math.hypot(x - robotPositionXYOptical[4], y - robotPositionXYOptical[5]);

            double absoluteAngleToTarget = Math.atan2(y - robotPositionXYOptical[5], x - robotPositionXYOptical[4]) * 180 / Math.PI;

            if (distanceToTarget < parkDistance && distanceToTarget > (parkDistance / 2)){
                robotTargetSpeed = movementSpeed * (distanceToTarget - (parkDistance / 2)) / 30;
                prevDistanceToTarget = parkDistance / 2;
            }else if (distanceToTarget < (parkDistance / 2)){
                robotTargetSpeedPID = NerdPID_PurePursuit.goToPositionParkPID(distanceToTarget, prevDistanceToTarget, loopTime);
                prevDistanceToTarget = distanceToTarget;
                robotTargetSpeed = robotTargetSpeedPID;
            }else{
                robotTargetSpeed = movementSpeed;
            }


            robotTargetAngle = absoluteAngleToTarget;

//            robotAngleToTarget = MathFunctions.AngleWrapDeg(robotTargetAngle - getAngle());
            motorAngleToTarget = MathFunctions.AngleWrapDeg((robotTargetAngle - 45) - getAngle());

            xPower = Math.cos(motorAngleToTarget * 3.14 / 180) * robotTargetSpeed;
            yPower = Math.sin(motorAngleToTarget * 3.14 / 180) * robotTargetSpeed;



//            double relativeTurnAngle = MathFunctions.AngleWrapDeg(robotAngleToTarget - 180 + preferredAngle);
            double relativeTurnAngle = MathFunctions.AngleWrapDeg(robotTargetAngle - (getAngle() + 90));

            if (Math.abs(preferredAngle - (getAngle() + 90)) > 35){
                useZPID = false;
            }
            else if (Math.abs(preferredAngle - (getAngle() + 90)) < 10){
                useZPID = true;
            }

            if (useZPID && distanceToTarget > parkDistance){
                zPIDAngle = 90 + getAngle();
                robotTurnSpeed = NerdPID_PurePursuit.zPowerDrive(preferredAngle, zPIDAngle, loopTime);
            } else if (useZPID && distanceToTarget < parkDistance){
                zPIDAngle = 90 + getAngle();
                robotTurnSpeed = NerdPID_PurePursuit.zPowerPark(preferredAngle, zPIDAngle, loopTime);
            } else if (!useZPID){
                robotTurnSpeed = Range.clip((preferredAngle - (getAngle() + 90)) / 30, -1, 1) * turnSpeed;
            }

            zPower = Range.clip(Math.abs(relativeTurnAngle / 30) * robotTurnSpeed, -0.4, 0.4);


            frontLeftMotorPower = -xPower + zPower;
            rearRightMotorPower = xPower + zPower;
            frontRightMotorPower = yPower + zPower;
            rearLeftMotorPower = -yPower + zPower;

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

            frontLeftMotor.setPower(motorSpeedCommand[0]);
            rearRightMotor.setPower(motorSpeedCommand[3]);
            frontRightMotor.setPower(motorSpeedCommand[1]);
            rearLeftMotor.setPower(motorSpeedCommand[2]);




//            if (debugFlag) {
//                RobotLog.d("goToPosition - timeSinceStart %f, robotTargetAngle %f, xPower %f, yPower %f , zPower %f, frontLeftMotorPower %f, rearRightMotorPower %f , frontRightMotorPower %f, rearLeftMotorPower %f, frontLeftMotorTicks %f, rearRightMotorTicks %f , frontRightMotorTicks %f , rearLeftMotorTicks %f, xPosition %f, yPosition %f, robotRot %f, robotRotDisplacement %f, robotAngleToTarget %f, robotVectorByOdoF %f, robotVectorByOdoR %f, frontVectorMag %f, rearVectorMag %f",
//                        deltaTime, robotTargetAngle, xPower, yPower, zPower, frontLeftMotorPower, rearRightMotorPower, frontRightMotorPower, rearLeftMotorPower, lfDisplacement, rrDisplacement, rfDisplacement, lrDisplacement, xPosition, yPosition, robotRot, robotRotDisplacement, robotAngleToTarget, robotVectorByOdoF, robotVectorByOdoR, frontVectorMag, rearVectorMag);
//            }

            // channels to record for velocity following
//            if (debugFlag) {
//                RobotLog.d("goToPosition - deltaTime %f, frontLeftMotorTarget %f, frontLeftMotorSpeed %f, frontRightMotorTarget %f, frontRightMotorSpeed %f, rearLeftMotorTarget %f, rearLeftMotorSpeed %f, rearRightMotorTarget %f, rearRightMotorSpeed %f, frontLeftMotorPower %f, frontRightMotorPower %f, rearLeftMotorPower %f, rearRightMotorPower %f, robotTargetAngle %f, robotAngleToTarget %f, zPIDAngle %f, relativeTurnAngle %f, xPosition %f, yPosition %f, frontOpticalEncoder %f, rightOpticalEncoder %f, leftOpticalEncoder %f, backOpticalEncoder %f, xPositionOpt %f, yPositionOpt %f, omniDriveFactorOpt %f",
//                        deltaTime, frontLeftMotorTarget, frontLeftMotorSpeed, frontRightMotorTarget, frontRightMotorSpeed, rearLeftMotorTarget, rearLeftMotorSpeed, rearRightMotorTarget, rearRightMotorSpeed, motorSpeedCommand [0], motorSpeedCommand [1], motorSpeedCommand [2], motorSpeedCommand [3], robotTargetAngle, robotAngleToTarget, zPIDAngle, relativeTurnAngle, robotPositionXY[0], robotPositionXY[1], robotPositionXYOptical[0], robotPositionXYOptical [1], robotPositionXYOptical [2], robotPositionXYOptical[3], robotPositionXYOptical[4], robotPositionXYOptical[5], omniDriveFactorOpt);
//            }

            // channels to record to debug optical encoder field centric driving
            if (debugFlag) {
                RobotLog.d("goToPosition - runTime %f, deltaTime %f, robotXdisplacementOpt %f, robotYdisplacementOpt %f, robotVectorByOdoOpt %f, robotVectorByOdo %f, robotVectorMagOpt %f, robotVectorMag %f, robotFieldAngleOpt %f, robotFieldAngle %f, relativeTurnAngle %f, robotAngleToTarget %f, robotTargetAngle %f, xPositionOpt %f, xPosition %f, yPositionOpt %f, yPosition %f, omniDriveFactorOpt %f, omniDriveFactor %f, distanceToTarget %f, robotTargetSpeed %f, robotTurnSpeed %f, zSpeedTargetAngle %f, zPIDAngle %f, distanceToTargetAngle %f",
                        currentTime, loopTime, robotXdisplacementOpt, robotYdisplacementOpt, robotVectorByOdoOpt, robotVectorByOdo, robotVectorMagOpt, robotVectorMag, robotFieldAngleOpt, robotFieldAngle, relativeTurnAngle, robotAngleToTarget, robotTargetAngle, xPositionOpt, xPosition, yPositionOpt, yPosition, omniDriveFactorOpt, omniDriveFactor, distanceToTarget, robotTargetSpeed, robotTurnSpeed, zSpeedTargetAngle, zPIDAngle, distanceToTargetAngle);

            }


        }

        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);

    }

    private double [] findDisplacement(double displacementX, double displacementY, double robotVector){

        robotRotNew = getAngle();
        robotRot = robotRotNew - robotRotOld;
        robotRotOld = robotRotNew;

        //robot rotation expressed in motor ticks (robot angle, ticks per rev, robot diameter, degrees per rev, wheel diameter
        robotRotDisplacement = robotRot * 540 * (20.25 / (360 * 3.543));

        rfDisplacementNew = frontRightMotor.getCurrentPosition();
        lfDisplacementNew = frontLeftMotor.getCurrentPosition();
        lrDisplacementNew = rearLeftMotor.getCurrentPosition();
        rrDisplacementNew = rearRightMotor.getCurrentPosition();

        rfDisplacement = rfDisplacementNew - rfDisplacementOld;
        lfDisplacement = lfDisplacementNew - lfDisplacementOld;
        lrDisplacement = lrDisplacementNew - lrDisplacementOld;
        rrDisplacement = rrDisplacementNew - rrDisplacementOld;

        rfDisplacementOld = rfDisplacementNew;
        lfDisplacementOld = lfDisplacementNew;
        lrDisplacementOld = lrDisplacementNew;
        rrDisplacementOld = rrDisplacementNew;

        lfDispNoRot = lfDisplacement - robotRotDisplacement;
        rrDispNoRot = rrDisplacement - robotRotDisplacement;
        rfDispNoRot = rfDisplacement - robotRotDisplacement;
        lrDispNoRot = lrDisplacement - robotRotDisplacement;

        lfDispNoRotTot += robotRotDisplacement;
        rrDispNoRotTot += robotRotDisplacement;
        rfDispNoRotTot += robotRotDisplacement;
        lrDispNoRotTot += robotRotDisplacement;

        omniDriveAngle = motorAngleToTarget;

        if (Math.abs(Math.cos(omniDriveAngle * Math.PI / 180)) > 0.707) {
            omniDriveFactor = Math.abs(Math.cos(omniDriveAngle * Math.PI / 180));
        }
        else if (Math.abs(Math.sin(omniDriveAngle * Math.PI / 180)) > 0.707) {
            omniDriveFactor = Math.abs(Math.sin(omniDriveAngle * Math.PI / 180));
        }
        else {
            omniDriveFactor = 1.0;
        }


        //calculate X displacement, and convert ticks to inches (3.543 = wheel diameter inches, 560 = ticks per wheel rot), and account for robot angle
        //robotXdisplacement = ((-rfDispNoRot - lfDispNoRot + rrDispNoRot + lrDispNoRot) / 4) * ((3.543 * Math.PI) / 540) / omniDriveFactor;
        //robotXdisplacement = (((rrDispNoRot - lfDispNoRot) / 2) + ((lrDispNoRot + rfDispNoRotTot) / 2)) * ((3.543 * Math.PI) / 540) / omniDriveFactor;
        robotXdisplacement = ((rrDispNoRot - lfDispNoRot) / 2) * ((3.543 * Math.PI) / 540);
        //calculate Y displacement, and convert ticks to inches (3.543 = wheel diameter inches, 560 = ticks per wheel rot), and account for robot angle
        //robotYdisplacement = ((rfDispNoRot - lfDispNoRot + rrDispNoRot - lrDispNoRot) / 4) * ((3.543 * Math.PI) / 540) / omniDriveFactor;
        robotYdisplacement = ((-lrDispNoRot + rfDispNoRot) / 2) * ((3.543 * Math.PI) / 540);


        robotVectorByOdo = 45 + Math.atan2(robotYdisplacement, robotXdisplacement) * 180 / Math.PI;

        robotVectorMag = Math.sqrt((robotXdisplacement * robotXdisplacement) + (robotYdisplacement * robotYdisplacement));

        robotFieldAngle = (robotVectorByOdo + getAngle());

        robotFieldPositionX = robotVectorMag * Math.cos(robotFieldAngle * Math.PI / 180);  //field position in inches
        robotFieldPositionY = robotVectorMag * Math.sin(robotFieldAngle * Math.PI / 180);  //field position in inches

        xPosition += robotFieldPositionX;
        yPosition += robotFieldPositionY;


        double [] displacement = {xPosition, yPosition, robotVectorByOdo};
        return displacement;

    }

    private double [] findDisplacementOptical(){

        //First, determine the robot z movement, so encoder ticks caused by z movement can be removed from x, y movement
        robotRotNewOpt = getAngle();
        robotRotOpt = robotRotNewOpt - robotRotOldOpt;
        robotRotOldOpt = robotRotNewOpt;

        //robot rotation (each loop) expressed in motor ticks (robot angle, ticks per degree robot rotation...determined through testing for each encoder wheel).
        //double robotRotDisplacementOptFront = robotRotOpt * 0; //21.390 ticks per degree of robot rotation
        double robotRotDisplacementOptRight = robotRotOpt * 112.695; //21.085each wheel is mounted slightly different on the bot
        double robotRotDisplacementOptLeft = robotRotOpt * 109.594; //21.318
        double robotRotDisplacementOptBack = robotRotOpt * (-1.012); //21.093

        //measure encoder position
        //frontPositionOptical = frontEncoder.getCurrentPosition();
        frontPositionOptical = 0;
        rightPositionOptical = rightEncoder.getCurrentPosition();
        leftPositionOptical = leftEncoder.getCurrentPosition();
        backPositionOptical = backEncoder.getCurrentPosition();

        //encoder ticks for each sensor, each loop
        //double frontDisplacement = frontPositionOptical - frontDisplacementOld;
        double leftDisplacement = leftPositionOptical - leftDisplacementOld;
        double rightDisplacement = rightPositionOptical - rightDisplacementOld;
        double rearDisplacement = backPositionOptical - rearDisplacementOld;

        frontDisplacementOld = frontPositionOptical;
        leftDisplacementOld = leftPositionOptical;
        rightDisplacementOld = rightPositionOptical;
        rearDisplacementOld = backPositionOptical;

        //Now, remove the ticks caused by z movement from the total encoder count, each loop
        //double frontDispNoRot = frontDisplacement - robotRotDisplacementOptFront;
        double leftDispNoRot = leftDisplacement - robotRotDisplacementOptRight;
        double rightDispNoRot = rightDisplacement - robotRotDisplacementOptLeft;
        double rearDispNoRot = rearDisplacement - robotRotDisplacementOptBack;

        //This section was created for debugging
        //frontDispNoRotTotOpt += frontDispNoRot;
        leftDispNoRotTotOpt += leftDispNoRot;
        rightDispNoRotTotOpt += rightDispNoRot;
        rearDispNoRotTotOpt += rearDispNoRot;

        //The optical encoders are mounted inline to the robot preferred direction, so we can just use robot angle to target...difference between robot target angle and gyro angle
//        omniDriveAngle = robotAngleToTarget + 45;

        //Determine the omni-Drive wheel effective gear ratio
        if (Math.abs(Math.cos(robotAngleToTarget * Math.PI / 180)) > 0.707) {
            omniDriveFactorOpt = Math.abs(Math.cos(robotAngleToTarget * Math.PI / 180));
        }
        else if (Math.abs(Math.sin(robotAngleToTarget * Math.PI / 180)) > 0.707) {
            omniDriveFactorOpt = Math.abs(Math.sin(robotAngleToTarget * Math.PI / 180));
        }
        else {
            omniDriveFactorOpt = 1.0;
        }

        //calculate X displacement, and convert ticks to inches (2.362 = wheel diameter inches, 1440 = ticks per wheel rot), and account for robot angle and omni wheel effect
//        robotXdisplacementOpt = ((-frontDispNoRot + rearDispNoRot) / 2) * ((2.362 * Math.PI) / 8192);
        robotXdisplacementOpt = rearDispNoRot * ((2.362 * 1.01 * Math.PI) / 8192); //added 2% error factor to improve accuracy.  Maybe the wheels are bigger than 60mm
//        robotXdisplacementOpt = ((-frontDispNoRot + rearDispNoRot) / 2) * ((2.362 * Math.PI) / 1440) / omniDriveFactorOpt;
        //robotXdisplacementOptTot = ((rightDispNoRotTotOpt + leftDispNoRotTotOpt) / 2 + (-frontDispNoRotTotOpt + rearDispNoRotTotOpt) / 2) * ((2.362 * 1.01 * Math.PI) / 8192);
        //calculate Y displacement, and convert ticks to inches (2.362 = wheel diameter inches, 1440 = ticks per wheel rot), and account for robot angle and omni wheel effect
        robotYdisplacementOpt = ((rightDispNoRot - leftDispNoRot) / 2) * ((2.362 * 1.01 * Math.PI) / 8192);
//        robotYdisplacementOpt = ((rightDispNoRot - leftDispNoRot) / 2) * ((2.362 * Math.PI) / 1440) / omniDriveFactorOpt;
        //robotYdisplacementOptTot = (((rightDispNoRotTotOpt - leftDispNoRotTotOpt) / 2) + ((frontDispNoRotTotOpt + rearDispNoRotTotOpt) / 2)) * ((2.362 * 1.01 * Math.PI) / 8192);

        //Using inverse kinematics, calculate the robot driving direction, from the encoder measurements
        robotVectorByOdoOpt = Math.atan2(robotYdisplacementOpt, robotXdisplacementOpt) * 180 / Math.PI;

        //Now that we know the robot driving direction, calculate the driving distance, each loop
        robotVectorMagOpt = Math.sqrt((robotXdisplacementOpt * robotXdisplacementOpt) + (robotYdisplacementOpt * robotYdisplacementOpt));

        //The calculated robot vector is the direction in field centric.  Adding the robot gyro angle was an error.
        robotFieldAngleOpt = (robotVectorByOdoOpt + getAngle());
        //robotFieldAngleOpt = robotVectorByOdoOpt;

        //Now we know the driving direction and distance for each loop, use forward kinematics calculation to determine x, y movement, each loop
        double robotFieldPositionXOpt = robotVectorMagOpt * Math.cos(robotFieldAngleOpt * Math.PI / 180);  //field position in inches
        double robotFieldPositionYOpt = robotVectorMagOpt * Math.sin(robotFieldAngleOpt * Math.PI / 180);  //field position in inches

        //Add each x, y loop calculation, to track the robot location on the field
        xPositionOpt += robotFieldPositionXOpt;
        yPositionOpt += robotFieldPositionYOpt;

        if (debugFlag) {
            RobotLog.d("findDisplacementOptical - runTime %f, deltaTime %f, leftDispNoRot %f, rightDispNoRot %f, rearDispNoRot %f, xPositionOptical %f, yPositionOptical %f, xPower %f, yPower %f, zPower %f, robotRotNewOpt %f, robotVectorByOdoOpt %f, robotFieldAngleOpt %f",
                    currentTime, loopTime, leftDispNoRot, rightDispNoRot, rearDispNoRot, xPositionOpt, yPositionOpt, xPower, yPower, zPower, robotRotNewOpt, robotVectorByOdoOpt, robotFieldAngleOpt);

        }

        //Store the encoder positions and x, y locations in an array and return the values
        double [] positionOptical = {frontPositionOptical, rightPositionOptical, leftPositionOptical, backPositionOptical, xPositionOpt, yPositionOpt, robotVectorByOdoOpt};
        return positionOptical;


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
        double wheelSpeedRPS = motorPower * 6000 / 60 / 19.2; //convert motor power to wheel rotations per second, 6000 rpm max motor speed, 60 seconds in a minute.
        double wheelSpeedIPS = wheelSpeedRPS * wheelDiameter * Math.PI; //
        return wheelSpeedIPS;

    }

    private double speedToPower (double wheelSpeed){
        double motorPower = wheelSpeed * 60 * 19.2 / 6000 / wheelDiameter / Math.PI;
//        double wheelSpeedRPS = motorPower * 6000 / 60 / 19.2; //convert motor power to wheel rotations per second, 6000 rpm max motor speed, 60 seconds in a minute.
//        double wheelSpeedIPS = wheelSpeedRPS * wheelDiameter * Math.PI; //
        return motorPower;

    }

    boolean distanceTargetReached(double distanceToTarget, double parkRadius){

        boolean onDistanceTarget = false;

        if (distanceToTarget < parkRadius){
            onDistanceTarget = true;
        }


        return  onDistanceTarget;
    }

    boolean distanceTargetReachedPark (double distanceToTarget, double deltaTime, double parkDistance, double parkTime){

        boolean onDistanceTarget = false;

        if (distanceToTarget < parkDistance){
            parkTimer += deltaTime;
        }

        if (distanceToTarget < parkDistance && parkTimer > parkTime){
            onDistanceTarget = true;
        }

        return  onDistanceTarget;
    }

    public void followCurve(ArrayList<CurvePoint> allPoints, double followAngle, double distanceToPark, double parkAngleTarget, double parkRadius){

        startTime = elapsedTime.seconds();
        oldTime = startTime;
        distanceToEndPoint = 10;
        NerdPID_PurePursuit.resetIntError();

        while (this.opmode.opModeIsActive() && !this.opmode.isStopRequested() && !distanceTargetReached(distanceToEndPoint, parkRadius)) {

            for (int i = 0; i < allPoints.size() - 1; i++) {
                ComputerDebugging.sendLine(new FloatPoint(allPoints.get(i).x, allPoints.get(i).y),
                        new FloatPoint(allPoints.get(i + 1).x, allPoints.get(i + 1).y));
            }

            double[] robotPositionXYV = findDisplacementOptical();

            CurvePoint followMe = getFollowPointPath(allPoints, new PointPP(robotPositionXYV[4], robotPositionXYV[5]),
                    allPoints.get(0).followDistance);

            CurvePoint endPoint = getEndPoint(allPoints, new PointPP(robotPositionXYV[4], robotPositionXYV[5]),
                    allPoints.get(0).followDistance);

            ComputerDebugging.sendKeyPoint(new FloatPoint(followMe.x, followMe.y));

            distanceToEndPoint = Math.hypot(endPoint.x - robotPositionXYV[4], endPoint.y - robotPositionXYV[5]);

            if(distanceToEndPoint < distanceToPark){
                goToPositionEndPP(endPoint.x, endPoint.y, 1.0, parkAngleTarget, 0.2, distanceToPark);
            }
            else {
                goToPositionPP(followMe.x, followMe.y, followMe.moveSpeed, followAngle, followMe.turnSpeed, parkAngleTarget);
            }

//            if (debugFlag) {
//                RobotLog.d("FollowCurve - distanceToEndPoint %f, endPoint.x %f, endPoint.y %f, followMe.x %f, followMe.y %f, robotXPosition %f, robotYPosition %f, followMe.moveSpeed %f, followAngle %f, followMe.turnspeed %f",
//                        distanceToEndPoint, endPoint.x, endPoint.y, followMe.x, followMe.y, robotPositionXYV[4], robotPositionXYV[5], followMe.moveSpeed, followAngle, followMe.turnSpeed);
//            }

        }
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);
    }

    private CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, PointPP robotLocation, double followRadius){
        CurvePoint followMe = new CurvePoint(pathPoints.get(0));

        for(int i = 0; i < pathPoints.size() - 1; i++){
            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endline = pathPoints.get(i + 1);

            double[] robotPositionXYV = findDisplacementOptical();

            ArrayList<PointPP> intersections = MathFunctions.lineCircleIntersection(robotLocation, followRadius, startLine.toPoint(), endline.toPoint());

            double closestAngle = 100000000;

            double closestDistance = 1000000;


            for(PointPP thisIntersection : intersections){
//                double angle = Math.atan2(thisIntersection.y - robotPositionXYV[5], thisIntersection.x - robotPositionXYV[4]);
//                //Need to check which angle is the same as worldAngle_rad...currently using getAngle() converted to radians
//                double deltaAngle = Math.abs(MathFunctions.AngleWrap(angle - (robotPositionXYV[6] * Math.PI / 180)));
//
//                if(deltaAngle < closestAngle){
//                    closestAngle = deltaAngle;
//                    followMe.setPoint(thisIntersection);
                double distance = Math.hypot(thisIntersection.x - robotPositionXYV[4], thisIntersection.y - robotPositionXYV[5]);

                if(distance < closestDistance){
                    closestDistance = distance;
                    followMe.setPoint(thisIntersection);
                }
            }
        }

//        if (debugFlag) {
//            RobotLog.d("CurvePoint - followMe.x %f, followMe.y %f, followMe.moveSpeed %f, followMe.turnSpeed %f, followRadius %f, pointLength %f, slowDownTurnAmount %f, slowDownTurnRadians %f",
//                    followMe.x, followMe.y, followMe.moveSpeed, followMe.turnSpeed, followRadius, followMe.pointLength, followMe.slowDownTurnAmount, followMe.slowDownTurnRadians);
//        }

        return followMe;
    }

    private CurvePoint getEndPoint(ArrayList<CurvePoint> pathPoints, PointPP robotLocation, double followRadius){

        int endPathCount = pathPoints.size() - 2;
        CurvePoint endPoint = new CurvePoint(pathPoints.get(endPathCount));

//        int pathCount = pathPoints.size();
//        CurvePoint startPath = pathPoints.get(0);
//        CurvePoint endPath = pathPoints.get(pathCount - 1);
//        ArrayList<PointPP> endPathPoint = MathFunctions.lineCircleIntersection(robotLocation, followRadius, startPath.toPoint(), endPath.toPoint());
//        for(PointPP thisEndPathPoint : endPathPoint){
//            endPoint.setPoint(thisEndPathPoint);
//        }


//        if (debugFlag) {
//            RobotLog.d("getEndPoint - followMe.x %f, followMe.y %f, followMe.moveSpeed %f, followMe.turnSpeed %f, followRadius %f, pointLength %f, slowDownTurnAmount %f, slowDownTurnRadians %f",
//                    followMe.x, followMe.y, followMe.moveSpeed, followMe.turnSpeed, followRadius, followMe.pointLength, followMe.slowDownTurnAmount, followMe.slowDownTurnRadians);
//        }

        return endPoint;
    }


    public void goToPositionPP(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed, double parkAngleTarget){
//        startTime = runtime;

//        distanceToTarget = 10;

//        while (this.opmode.opModeIsActive() && !this.opmode.isStopRequested()) {

        //First calculate motor speeds for linear (x, y) motion

        currentTime = elapsedTime.seconds();
        loopTime = currentTime - oldTime;
        oldTime = currentTime;
//        deltaTime = currentTime - startTime;

        robotPositionXY = findDisplacement(xPosition, yPosition, robotVectorByOdo);
        robotPositionXYOptical = findDisplacementOptical();

//        distanceToTarget = Math.hypot(x - robotPositionXY[0], y - robotPositionXY[1]);
        distanceToTarget = Math.hypot(x - robotPositionXYOptical[4], y - robotPositionXYOptical[5]);

        double absoluteAngleToTarget = Math.atan2(y - robotPositionXYOptical[5], x - robotPositionXYOptical[4]) * 180 / Math.PI;

//            if (distanceToTarget < 5) {
//                robotTargetSpeed = 0;
//            } else {
//                robotTargetSpeed = movementSpeed;
//            }

        robotTargetAngle = absoluteAngleToTarget;

//        robotAngleToTarget = MathFunctions.AngleWrapDeg(robotTargetAngle - getAngle());
        motorAngleToTarget = MathFunctions.AngleWrapDeg((robotTargetAngle - 45) - getAngle());

        xPower = Math.cos(motorAngleToTarget * 3.14 / 180) * movementSpeed;
        yPower = Math.sin(motorAngleToTarget * 3.14 / 180) * movementSpeed;

        double relativeTurnAngle = MathFunctions.AngleWrapDeg(robotTargetAngle - (getAngle() + 90));

        if (Math.abs(parkAngleTarget - (getAngle() + 90)) > 90){
            useZPID = false;
        }
        else if (Math.abs(parkAngleTarget - (getAngle() + 90)) < 80){
            useZPID = true;
        }

        if (useZPID){
            zPIDAngle = 90 + getAngle();
            robotTurnSpeed = NerdPID_PurePursuit.zPowerDrive(parkAngleTarget, zPIDAngle, loopTime);
        }
        else if (!useZPID){
            robotTurnSpeed = Range.clip((parkAngleTarget - (getAngle() + 90)) / 30, -1, 1) * turnSpeed;
        }

        double robotTurnSpeedFF = Range.clip((zPowerStart + zPowerIncrease), -0.5, 0);
        zPowerStart = robotTurnSpeedFF;

//        zPower = Range.clip(relativeTurnAngle / 30, -0.3, 0.3) * robotTurnSpeed;
        zPower = Range.clip((robotTurnSpeed + zPowerStart), -0.5, 0.5);

        //Second calculate motor speeds for angular (z) motion

//            robotCircumference = 2 * Math.PI * robotRadius;
//            robotWheelCircumference = 2 * Math.PI * robotWheelRadius;
//            wheelRotPerRobotRot = robotCircumference / robotWheelCircumference;

        frontLeftMotorPower = -xPower + zPower;
        rearRightMotorPower = xPower + zPower;
        frontRightMotorPower = yPower + zPower;
        rearLeftMotorPower = -yPower + zPower;

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

        frontLeftMotor.setPower(motorSpeedCommand[0]);
        rearRightMotor.setPower(motorSpeedCommand[3]);
        frontRightMotor.setPower(motorSpeedCommand[1]);
        rearLeftMotor.setPower(motorSpeedCommand[2]);




//            if (debugFlag) {
//                RobotLog.d("goToPositionPP - timeSinceStart %f, robotTargetAngle %f, xPower %f, yPower %f , zPower %f, frontLeftMotorPower %f, rearRightMotorPower %f , frontRightMotorPower %f, rearLeftMotorPower %f, frontLeftMotorTicks %f, rearRightMotorTicks %f , frontRightMotorTicks %f , rearLeftMotorTicks %f, xPosition %f, yPosition %f, robotRot %f, robotRotDisplacement %f, robotAngleToTarget %f, robotVectorByOdoF %f, robotVectorByOdoR %f, frontVectorMag %f, rearVectorMag %f",
//                        deltaTime, robotTargetAngle, xPower, yPower, zPower, frontLeftMotorPower, rearRightMotorPower, frontRightMotorPower, rearLeftMotorPower, lfDisplacement, rrDisplacement, rfDisplacement, lrDisplacement, xPosition, yPosition, robotRot, robotRotDisplacement, robotAngleToTarget, robotVectorByOdoF, robotVectorByOdoR, frontVectorMag, rearVectorMag);
//            }

        if (debugFlag) {
            RobotLog.d("goToPositionPP - runTime %f, deltaTime %f,robotPositionXY[0] %f, robotPositionXY[1] %f, distanceToTarget %f, robotAngleToTarget %f, robotVectorByOdo %f, robotVectorMag %f, robotFieldAngle %f, omniDriveAngle %f, xPower %f, yPower %f, zPower %f, relativeTurnAngle %f, motorAngleToTarget %f, robotTargetAngle %f, getAngle() %f, frontLeftMotorSpeed %f, frontLeftMotorPower %f, frontRightMotorSpeed %f, frontRightMotorPower%f",
                    currentTime, deltaTime, robotPositionXYOptical[4], robotPositionXYOptical[5], distanceToTarget, robotAngleToTarget, robotVectorByOdo, robotVectorMag, robotFieldAngle, omniDriveAngle, xPower, yPower, zPower, relativeTurnAngle, motorAngleToTarget, robotTargetAngle, getAngle(), frontLeftMotorSpeed, motorSpeedCommand[0], frontRightMotorSpeed, motorSpeedCommand[1]);
        }






//        frontLeftMotor.setPower(0);
//        frontRightMotor.setPower(0);
//        rearLeftMotor.setPower(0);
//        rearRightMotor.setPower(0);

    }

    public void goToPositionEndPP(double x, double y, double movementSpeed, double targetAngleForPark, double turnSpeed, double distanceToEndPoint){
//        startTime = runtime;

//        distanceToTarget = 10;
        distanceToTargetAngle = getAngle() + 90;

//        double [] robotPositionXYOpticalInit = findDisplacementOptical();
//        xPositionOpticalInit = robotPositionXYOpticalInit[4];
//        yPositionOpticalInit = robotPositionXYOpticalInit[5];

        //First calculate motor speeds for linear (x, y) motion

        currentTime = elapsedTime.seconds();
        loopTime = currentTime - oldTime;
        oldTime = currentTime;
//        deltaTime = currentTime - startTime;

        robotPositionXY = findDisplacement(xPosition, yPosition, robotVectorByOdo);

        robotPositionXYOptical = findDisplacementOptical();

        //distanceToTarget = Math.hypot(x - robotPositionXY[0], y - robotPositionXY[1]);
        distanceToTarget = Math.hypot(x - robotPositionXYOptical[4], y - robotPositionXYOptical[5]);

        double absoluteAngleToTarget = Math.atan2(y - robotPositionXYOptical[5], x - robotPositionXYOptical[4]) * 180 / Math.PI;

        if (distanceToEndPoint > 25) {
            if (distanceToTarget < distanceToEndPoint && distanceToTarget > (distanceToEndPoint / 2)) {
                robotTargetSpeed = Range.clip(movementSpeed * (distanceToTarget - (distanceToEndPoint / 2)) / 30, 0.2, 1);
                prevDistanceToTarget = distanceToEndPoint / 2;
            } else if (distanceToTarget < (distanceToEndPoint / 2)) {
                robotTargetSpeedPID = NerdPID_PurePursuit.movementSpeedPID(distanceToTarget, prevDistanceToTarget, loopTime);
                prevDistanceToTarget = distanceToTarget;
                robotTargetSpeed = robotTargetSpeedPID;
            }
        }else{
            robotTargetSpeedPID = NerdPID_PurePursuit.shortParkPID(distanceToTarget, prevDistanceToTarget, loopTime);
            prevDistanceToTarget = distanceToTarget;
            robotTargetSpeed = robotTargetSpeedPID;
        }

        robotTargetAngle = absoluteAngleToTarget;

        robotAngleToTarget = MathFunctions.AngleWrapDeg(robotTargetAngle - getAngle());
        motorAngleToTarget = MathFunctions.AngleWrapDeg((robotTargetAngle - 45) - getAngle());

        xPower = Math.cos(motorAngleToTarget * 3.14 / 180) * robotTargetSpeed;
        yPower = Math.sin(motorAngleToTarget * 3.14 / 180) * robotTargetSpeed;



//            double relativeTurnAngle = MathFunctions.AngleWrapDeg(robotAngleToTarget - 180 + preferredAngle);
        double relativeTurnAngle = MathFunctions.AngleWrapDeg(targetAngleForPark - 90 - getAngle());
        double targetParkAngle = targetAngleForPark;
        zPIDAngle = 90 + getAngle();
        zPower = NerdPID_PurePursuit.zPowerPark(targetParkAngle, zPIDAngle, loopTime);

        frontLeftMotorPower = -xPower + zPower;
        rearRightMotorPower = xPower + zPower;
        frontRightMotorPower = yPower + zPower;
        rearLeftMotorPower = -yPower + zPower;

        if(Math.abs(frontLeftMotorPower) > Math.abs(frontRightMotorPower) && Math.abs(frontLeftMotorPower) > Math.abs(rearLeftMotorPower) && Math.abs(frontLeftMotorPower) > Math.abs(rearRightMotorPower)){
            maxPowerEndPP = Math.abs(frontLeftMotorPower);
        }
        else if (Math.abs(frontRightMotorPower) > Math.abs(frontLeftMotorPower) && Math.abs(frontRightMotorPower) > Math.abs(rearLeftMotorPower) && Math.abs(frontRightMotorPower) > Math.abs(rearRightMotorPower)){
            maxPowerEndPP = Math.abs(frontRightMotorPower);
        }
        else if(Math.abs(rearRightMotorPower) > Math.abs(frontRightMotorPower) && Math.abs(rearRightMotorPower) > Math.abs(rearLeftMotorPower) && Math.abs(rearRightMotorPower) > Math.abs(frontLeftMotorPower)){
            maxPowerEndPP = Math.abs(rearRightMotorPower);
        }
        if(Math.abs(rearLeftMotorPower) > Math.abs(frontRightMotorPower) && Math.abs(rearLeftMotorPower) > Math.abs(frontLeftMotorPower) && Math.abs(rearLeftMotorPower) > Math.abs(rearRightMotorPower)) {
            maxPowerEndPP = Math.abs(rearLeftMotorPower);
        }

        if(maxPowerEndPP > 1) {
            frontLeftMotorPower /= maxPowerEndPP;
            frontRightMotorPower /= maxPowerEndPP;
            rearLeftMotorPower /= maxPowerEndPP;
            rearRightMotorPower /= maxPowerEndPP;
        }

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

        frontLeftMotor.setPower(motorSpeedCommand[0]);
        rearRightMotor.setPower(motorSpeedCommand[3]);
        frontRightMotor.setPower(motorSpeedCommand[1]);
        rearLeftMotor.setPower(motorSpeedCommand[2]);




//            if (debugFlag) {
//                RobotLog.d("goToPosition - timeSinceStart %f, robotTargetAngle %f, xPower %f, yPower %f , zPower %f, frontLeftMotorPower %f, rearRightMotorPower %f , frontRightMotorPower %f, rearLeftMotorPower %f, frontLeftMotorTicks %f, rearRightMotorTicks %f , frontRightMotorTicks %f , rearLeftMotorTicks %f, xPosition %f, yPosition %f, robotRot %f, robotRotDisplacement %f, robotAngleToTarget %f, robotVectorByOdoF %f, robotVectorByOdoR %f, frontVectorMag %f, rearVectorMag %f",
//                        deltaTime, robotTargetAngle, xPower, yPower, zPower, frontLeftMotorPower, rearRightMotorPower, frontRightMotorPower, rearLeftMotorPower, lfDisplacement, rrDisplacement, rfDisplacement, lrDisplacement, xPosition, yPosition, robotRot, robotRotDisplacement, robotAngleToTarget, robotVectorByOdoF, robotVectorByOdoR, frontVectorMag, rearVectorMag);
//            }

        // channels to record for velocity following
//            if (debugFlag) {
//                RobotLog.d("goToPosition - deltaTime %f, frontLeftMotorTarget %f, frontLeftMotorSpeed %f, frontRightMotorTarget %f, frontRightMotorSpeed %f, rearLeftMotorTarget %f, rearLeftMotorSpeed %f, rearRightMotorTarget %f, rearRightMotorSpeed %f, frontLeftMotorPower %f, frontRightMotorPower %f, rearLeftMotorPower %f, rearRightMotorPower %f, robotTargetAngle %f, robotAngleToTarget %f, zPIDAngle %f, relativeTurnAngle %f, xPosition %f, yPosition %f, frontOpticalEncoder %f, rightOpticalEncoder %f, leftOpticalEncoder %f, backOpticalEncoder %f, xPositionOpt %f, yPositionOpt %f, omniDriveFactorOpt %f",
//                        deltaTime, frontLeftMotorTarget, frontLeftMotorSpeed, frontRightMotorTarget, frontRightMotorSpeed, rearLeftMotorTarget, rearLeftMotorSpeed, rearRightMotorTarget, rearRightMotorSpeed, motorSpeedCommand [0], motorSpeedCommand [1], motorSpeedCommand [2], motorSpeedCommand [3], robotTargetAngle, robotAngleToTarget, zPIDAngle, relativeTurnAngle, robotPositionXY[0], robotPositionXY[1], robotPositionXYOptical[0], robotPositionXYOptical [1], robotPositionXYOptical [2], robotPositionXYOptical[3], robotPositionXYOptical[4], robotPositionXYOptical[5], omniDriveFactorOpt);
//            }

//            // channels to record to debug optical encoder field centric driving
//            if (debugFlag) {
//                RobotLog.d("goToPositionEndPP - deltaTime %f, robotXdisplacementOpt %f, robotYdisplacementOpt %f, robotVectorByOdoOpt %f, robotVectorByOdo %f, robotVectorMagOpt %f, robotVectorMag %f, robotFieldAngleOpt %f, robotFieldAngle %f, relativeTurnAngle %f, robotAngleToTarget %f, robotTargetAngle %f, xPositionOpt %f, xPosition %f, yPositionOpt %f, yPosition %f, omniDriveFactorOpt %f, omniDriveFactor %f, distanceToTarget %f, robotTargetSpeed %f, zPIDPower %f, zSpeedTargetAngle %f, zPIDAngle %f, distanceToTargetAngle %f",
//                        deltaTime, robotXdisplacementOpt, robotYdisplacementOpt, robotVectorByOdoOpt, robotVectorByOdo, robotVectorMagOpt, robotVectorMag, robotFieldAngleOpt, robotFieldAngle, relativeTurnAngle, robotAngleToTarget, robotTargetAngle, xPositionOpt, xPosition, yPositionOpt, yPosition, omniDriveFactorOpt, omniDriveFactor, distanceToTarget, robotTargetSpeed, zPower, zSpeedTargetAngle, zPIDAngle, distanceToTargetAngle);
//
//            }

        // channels to record to debug optical encoder field centric driving
        if (debugFlag) {
            RobotLog.d("goToPositionEndPP - runTime %f, deltaTime %f, distanceToTarget %f, robotTargetSpeed %f, robotTargetSpeedIPS %f, robotSpeed %f, decelRate %f, xPositionOptical %f, yPositionOptical %f, frontLeftMotorPower %f, rearRightMotorPower %f, frontRightMotorPower %f, rearLeftMotorPower %f, targetParkAngle %f, zPIDAngle %f, xPower %f, yPower %f, zPower %f, robotTargetSpeedPID %f, prevDistanceToTarget %f",
                    currentTime, loopTime, distanceToTarget, robotTargetSpeed, robotTargetSpeedIPS, robotSpeed, decelRate, xPositionOpt, yPositionOpt, motorSpeedCommand[0], motorSpeedCommand[3], motorSpeedCommand[1], motorSpeedCommand[2], targetParkAngle, zPIDAngle, xPower, yPower, zPower, robotTargetSpeedPID, prevDistanceToTarget);

        }

    }

    //    Puts the wobble arm down from it's original position. Used at the start of the match.
    //    leftEncoder = wobbleMotor
    public void beginningDown() {
        Timer.reset();
        while(Timer.seconds() < 0.35) {
            leftEncoder.setPower(0.55);
        }
        leftEncoder.setPower(0);

        wobbleServo.setPosition(0);

        opmode.sleep(500);

    }

    //Closes servo, then picks up wobble goal
    public void pickupWobble() {
        wobbleServo.setPosition(0.75);

        opmode.sleep(1000);

        Timer.reset();
        while(Timer.seconds() < 0.75) {
            leftEncoder.setPower(-0.9);
        }
        leftEncoder.setPower(0.1);

    }
    //Lowers motor, then releases wobble goal
    public void setDownWobble() {
        Timer.reset();
        while(Timer.seconds() < 0.35) {
            leftEncoder.setPower(0.55);
        }

        leftEncoder.setPower(0);
        opmode.sleep(500);
        wobbleServo.setPosition(0);
    }

    public void upWobble() {
        wobbleServo.setPosition(0.75);

        opmode.sleep(1000);

        Timer.reset();
        while(Timer.seconds() < 0.75) {
            leftEncoder.setPower(-0.9);
        }
        leftEncoder.setPower(0.0);
    }

//    public void runShoot() {
//        runShooterMotor(0.92);
//        opmode.sleep(1000);
//        indexRings();
//
//    }
//
//    //Function to run shooter motor. Run this function and then sleep for a little bit to let
//    //the motor charge up. Then run the indexRings function
//    //frontEncoder = shooterMotor and rightEncoder = intakeMotor
//    public void runShooterMotor(double motorPower) {
//        frontEncoder.setPower(motorPower);
//    }

//    public void indexRings() {
//        int count = 0;
//        boolean cycle = true;
//        indexingServo.setPosition(indexerHomePos);
//        while(count < 8){
//            double position = 0.0;
//
//            if (cycle) {
//                //Make Indexer go forward
//                position = this.indexerPushedPos;
//                cycle = false;
//            } else {
//                //Make Indexer go backward
//                position = this.indexerHomePos;
//                cycle = true;
//            }
//
//            if(count == 6 || count == 7) {
//                rightEncoder.setPower(-1);
//            }
//
//            // Set the servo to the new position and pause;
//            indexingServo.setPosition(position);
//            opmode.sleep(300);
//            //sleep(1000);
//            count++;
//        }
//        //Stop Motor
//        frontEncoder.setPower(0);
//        rightEncoder.setPower(0);
//    }

    //Function to run shooter motor. Run this function and then sleep for a little bit to let
//the motor charge up. Then run the indexRings function
    public void runShoot() {
        this.frontEncoder.setVelocity(shooterVeloc);
        indexRings();
    }
    public void indexRings() {
        boolean shouldIndex = false;
        int count = 0;
        boolean cycle = true;
        indexingServo.setPosition(indexerHomePos);
        while(count < 8){
            double position = 0.0;
            if(shouldIndex) {
                if (cycle) {
                    //Make Indexer go forward
                    position = this.indexerPushedPos;
                    cycle = false;
                } else {
                    //Make Indexer go backward
                    position = this.indexerHomePos;
                    cycle = true;
                }
                if (count == 6 || count == 7) {
                    this.rightEncoder.setPower(-1);
                }
            }
            // Set the servo to the new position and pause;
            indexingServo.setPosition(position);
            //checks if the velocity is within a threshold
            if((Math.abs(Math.abs(frontEncoder.getVelocity()) - Math.abs(shooterVeloc))) < 100) {
                shouldIndex = true;
                count++;
            } else {
                shouldIndex = false;
            }
        }
        //Stop Motor
        this.frontEncoder.setVelocity(0);
        //this.frontEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        this.rightEncoder.setPower(0);
    }



}




