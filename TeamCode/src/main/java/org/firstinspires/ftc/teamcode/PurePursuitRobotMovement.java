//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//import com.qualcomm.robotcore.util.RobotLog;
//
//import company.ComputerDebugging;
//import company.FloatPoint;
//import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.robotcore.external.navigation.Position;
//import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
//import opencv.core.Point;
//import treamcode.CurvePoint;
//import treamcode.MathFunctions;
//import treamcode.NerdPIDCalc_MotionProfiling_Clean;
//import treamcode.NerdHardwareWorld;
//
//import java.util.ArrayList;
//
//public class PurePursuitRobotMovement {
//
//    private boolean debugFlag=false;
//
//    //We need an opmode to get the hardware map etc.
//
//    private LinearOpMode opmode;
//
//    private HardwareMap hardwareMap;
//
//    private DcMotor frontLeftMotor;
//    private DcMotor frontRightMotor;
//    private DcMotor rearLeftMotor;
//    private DcMotor rearRightMotor;
//
//    private ElapsedTime runtime = new ElapsedTime();
//    private ElapsedTime elapsedTime = new ElapsedTime();
//    private ElapsedTime settlingtime = new ElapsedTime();
//
//    private BNO055IMU imu = null;   // Gyro device
//
//    Orientation angles;
//    Acceleration gravity;
//
//    Orientation lastAngles = new Orientation();
//
//    private NerdPIDCalc_MotionProfiling_Clean Motion_PID;
//
//
//    private double robotAngleToField = 0;
//    private double robotAngleToTarget = 0;
//
//    private double xPosition = 0;
//    private double yPosition = 0;
//    private double displacementX = 0;
//    private double displacementY = 0;
//
//    double robotRot = 0;
//    double robotRotNew = 0;
//    double robotRotOld = 0;
//    double robotRotDisplacement = 0;
//    double robotVectorByOdo = 0;
//    double rfDisplacement = 0;
//    double rfDisplacementNew = 0;
//    double rfDisplacementOld = 0;
//    double lfDisplacement = 0;
//    double lfDisplacementNew = 0;
//    double lfDisplacementOld = 0;
//    double rrDisplacement = 0;
//    double rrDisplacementNew = 0;
//    double rrDisplacementOld = 0;
//    double lrDisplacement = 0;
//    double lrDisplacementNew = 0;
//    double lrDisplacementOld = 0;
//    double rfDispNoRot = 0;
//    double lfDispNoRot = 0;
//    double rrDispNoRot = 0;
//    double lrDispNoRot = 0;
//    double rfDispNoRotTot = 0;
//    double lfDispNoRotTot = 0;
//    double rrDispNoRotTot = 0;
//    double lrDispNoRotTot = 0;
//    double robotVectorMag = 0;
//    double omniDriveAngle = 0;
//    double omniDriveFactor = 0;
//
//    double robotFieldPositionX = 0;
//    double robotFieldPositionY = 0;
//    double robotFieldAngle = 0;
//    double [] robotPositionXY;
//    double robotXdisplacement = 0;
//    double robotYdisplacement = 0;
//
//    double distanceToTarget = 10;
//
////    static final double DISTANCE_THRESHOLD = 2;
//
//    double deltaTime = 0;
//    double startTime = 0;
//    double oldTime = 0;
//    double loopTime = 0;
//    double currentTime = 0;
//
//    double xPower = 0;
//    double yPower = 0;
//    double zPower = 0;
//
//    double robotTargetSpeed = 0.5;
//    double robotTargetAngle = 0;
//    double robotTurnSpeed = 0;
//
//    double frontLeftMotorPower = 0;
//    double frontRightMotorPower = 0;
//    double rearLeftMotorPower = 0;
//    double rearRightMotorPower = 0;
//
//    NerdHardwareWorld HWorld;
//
//    /**
//     * Constructor to create NerdBOT object
//     * <p>
//     * Creates a new NerdBOT object and assigns the hardwareMap provided by caller
//     *
//     * @param opmode Hardware Map provided by the calling OpMode.
//     *               NerdBOT takes an opmode object so that it can get the hardwareMap.     *
//     */
//
//    public PurePursuitRobotMovement(LinearOpMode opmode) {
//        this.opmode = opmode;
//        this.hardwareMap = opmode.hardwareMap;
//        this.Motion_PID = new NerdPIDCalc_MotionProfiling_Clean(opmode);
//    }
//
//    //Function to initialize hardware components.
//
//    public void setDebug(boolean debugFlag){
//        this.debugFlag=debugFlag;
//    }
//
//
//    public void initializeHardware(){
//
//        HWorld = new NerdHardwareWorld();
//
//        this.imu = HWorld.imu;
//        this.frontLeftMotor = HWorld.FLM;
//        this.frontRightMotor = HWorld.FRM;
//        this.rearLeftMotor = HWorld.RLM;
//        this.rearRightMotor = HWorld.RRM;
//
//        resetAngle();
//
//        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
//
//        this.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        this.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        this.rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        this.rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        this.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        this.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        this.rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        this.rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        this.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        this.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        this.rearLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        this.rearRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        xPosition = 0;
//        yPosition = 0;
//
//        displacementX = 0;
//        displacementY = 0;
//
//
//
//    }
//
//
//    private void resetAngle() {
//
//
//        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//        robotAngleToField = 0;
//    }
//
//    //Function to get the angle of the Gyro sensor
//    private double getAngle() {
//
//        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
//
//        if (deltaAngle < -180)
//            deltaAngle += 360;
//        else if (deltaAngle > 180)
//            deltaAngle -= 360;
//
//        robotAngleToField += deltaAngle;
//
//        lastAngles = angles;
//
//        return robotAngleToField;
//    }
//
//    public void goToPosition(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed){
////        startTime = runtime;
//
//        distanceToTarget = 10;
//
//        while (this.opmode.opModeIsActive() && !this.opmode.isStopRequested() && !distanceTargetReached(distanceToTarget)) {
//
//            //First calculate motor speeds for linear (x, y) motion
//
////            currentTime = runtime;
////            loopTime = currentTime - oldTime;
////            oldTime = currentTime;
////            deltaTime = currentTime - startTime;
//
//            robotPositionXY = findDisplacement(xPosition, yPosition, robotVectorByOdo);
//
//            distanceToTarget = Math.hypot(x - robotPositionXY[0], y - robotPositionXY[1]);
//
//            double absoluteAngleToTarget = Math.atan2(y - robotPositionXY[1], x - robotPositionXY[0]) * 180 / Math.PI;
//
//            if (distanceToTarget < 5) {
//                robotTargetSpeed = 0;
//            } else {
//                robotTargetSpeed = movementSpeed;
//            }
//
//
//            robotTargetAngle = absoluteAngleToTarget;
//
//            robotAngleToTarget = MathFunctions.AngleWrapDeg((robotTargetAngle - 45) - getAngle());
//
//            xPower = Math.cos(robotAngleToTarget * 3.14 / 180) * robotTargetSpeed;
//            yPower = Math.sin(robotAngleToTarget * 3.14 / 180) * robotTargetSpeed;
//
//
//            double relativeTurnAngle = MathFunctions.AngleWrapDeg(robotAngleToTarget - 180 + preferredAngle);
//
//            if (distanceToTarget < 10) {
//                robotTurnSpeed = 0;
//            }else{
//                robotTurnSpeed = turnSpeed;
//            }
//
//            zPower = Range.clip(relativeTurnAngle / 30, -1, 1) * robotTurnSpeed;
//
//
//            //Second calculate motor speeds for angular (z) motion
//
////            robotCircumference = 2 * Math.PI * robotRadius;
////            robotWheelCircumference = 2 * Math.PI * robotWheelRadius;
////            wheelRotPerRobotRot = robotCircumference / robotWheelCircumference;
//
//            frontLeftMotorPower = -xPower + zPower;
//            rearRightMotorPower = xPower + zPower;
//            frontRightMotorPower = yPower + zPower;
//            rearLeftMotorPower = -yPower + zPower;
//
////            rearRightMotor.setPower(rearRightMotorPower);
////            frontLeftMotor.setPower(frontLeftMotorPower);
////            frontRightMotor.setPower(frontRightMotorPower);
////            rearLeftMotor.setPower(rearLeftMotorPower);
//
//            Motion_PID.runMotors(frontLeftMotorPower, frontRightMotorPower, rearLeftMotorPower, rearRightMotorPower, 0.75, 0.75);
//
//
//
//
////            if (debugFlag) {
////                RobotLog.d("FieldCentricInAutonTurn3Odo1 - timeSinceStart %f, robotTargetAngle %f, xPower %f, yPower %f , zPower %f, frontLeftMotorPower %f, rearRightMotorPower %f , frontRightMotorPower %f, rearLeftMotorPower %f, frontLeftMotorTicks %f, rearRightMotorTicks %f , frontRightMotorTicks %f , rearLeftMotorTicks %f, xPosition %f, yPosition %f, robotRot %f, robotRotDisplacement %f, robotAngleToTarget %f, robotVectorByOdoF %f, robotVectorByOdoR %f, frontVectorMag %f, rearVectorMag %f",
////                        deltaTime, robotTargetAngle, xPower, yPower, zPower, frontLeftMotorPower, rearRightMotorPower, frontRightMotorPower, rearLeftMotorPower, lfDisplacement, rrDisplacement, rfDisplacement, lrDisplacement, xPosition, yPosition, robotRot, robotRotDisplacement, robotAngleToTarget, robotVectorByOdoF, robotVectorByOdoR, frontVectorMag, rearVectorMag);
////            }
//
//            if (debugFlag) {
//                RobotLog.d("FieldCentricInAutonTurn3Odo1 - timeSinceStart %f, robotTargetAngle %f, lfDisplacementNew %f, lfDispNoRot %f, rrDisplacementNew %f, rrDispNoRot %f, rfDisplacementNew %f, rfDispNoRot %f, lrDisplacementNew %f, lrDispNoRot %f, xPosition %f, yPosition %f, robotRot %f, robotRotDisplacement %f, robotAngleToTarget %f, robotVectorByOdo %f, robotVectorMag %f, robotFieldAngle %f, robotFieldPositionX %f, robotFieldPositionY %f, robotXdisplacement %f, robotYdisplacement %f, omniDriveAngle %f, omniDriveFactor %f",
//                        deltaTime, robotTargetAngle, lfDisplacementNew, lfDispNoRot, rrDisplacementNew, rrDispNoRot, rfDisplacementNew, rfDispNoRot, lrDisplacementNew, lrDispNoRot, xPosition, yPosition, robotRot, robotRotDisplacement, robotAngleToTarget, robotVectorByOdo, robotVectorMag, robotFieldAngle, robotFieldPositionX, robotFieldPositionY, robotXdisplacement, robotYdisplacement, omniDriveAngle, omniDriveFactor);
//            }
//
//
//
//
//        }
//
//        frontLeftMotor.setPower(0);
//        frontRightMotor.setPower(0);
//        rearLeftMotor.setPower(0);
//        rearRightMotor.setPower(0);
//
//    }
//
//    private double [] findDisplacement(double displacementX, double displacementY, double robotVector){
//
//        robotRotNew = getAngle();
//        robotRot = robotRotNew - robotRotOld;
//        robotRotOld = robotRotNew;
//
//        //robot rotation expressed in motor ticks (robot angle, ticks per rev, robot diameter, degrees per rev, wheel diameter
//        robotRotDisplacement = robotRot * 540 * (20.25 / (360 * 3.543));
//
//        rfDisplacementNew = frontRightMotor.getCurrentPosition();
//        lfDisplacementNew = frontLeftMotor.getCurrentPosition();
//        lrDisplacementNew = rearLeftMotor.getCurrentPosition();
//        rrDisplacementNew = rearRightMotor.getCurrentPosition();
//
//        rfDisplacement = rfDisplacementNew - rfDisplacementOld;
//        lfDisplacement = lfDisplacementNew - lfDisplacementOld;
//        lrDisplacement = lrDisplacementNew - lrDisplacementOld;
//        rrDisplacement = rrDisplacementNew - rrDisplacementOld;
//
//        rfDisplacementOld = rfDisplacementNew;
//        lfDisplacementOld = lfDisplacementNew;
//        lrDisplacementOld = lrDisplacementNew;
//        rrDisplacementOld = rrDisplacementNew;
//
//        lfDispNoRot = lfDisplacement - robotRotDisplacement;
//        rrDispNoRot = rrDisplacement - robotRotDisplacement;
//        rfDispNoRot = rfDisplacement - robotRotDisplacement;
//        lrDispNoRot = lrDisplacement - robotRotDisplacement;
//
//        lfDispNoRotTot += robotRotDisplacement;
//        rrDispNoRotTot += robotRotDisplacement;
//        rfDispNoRotTot += robotRotDisplacement;
//        lrDispNoRotTot += robotRotDisplacement;
//
//        omniDriveAngle = robotAngleToTarget + 45;
//
//        if (Math.abs(Math.cos(omniDriveAngle * Math.PI / 180)) > 0.707) {
//            omniDriveFactor = Math.abs(Math.cos(omniDriveAngle * Math.PI / 180));
//        }
//        else if (Math.abs(Math.sin(omniDriveAngle * Math.PI / 180)) > 0.707) {
//            omniDriveFactor = Math.abs(Math.sin(omniDriveAngle * Math.PI / 180));
//        }
//        else {
//            omniDriveFactor = 1.0;
//        }
//
//
//        //calculate X displacement, and convert ticks to inches (3.543 = wheel diameter inches, 560 = ticks per wheel rot), and account for robot angle
//        robotXdisplacement = ((-rfDispNoRot - lfDispNoRot + rrDispNoRot + lrDispNoRot) / 4) * ((3.543 * Math.PI) / 540) / omniDriveFactor;
//        //calculate Y displacement, and convert ticks to inches (3.543 = wheel diameter inches, 560 = ticks per wheel rot), and account for robot angle
//        robotYdisplacement = ((rfDispNoRot - lfDispNoRot + rrDispNoRot - lrDispNoRot) / 4) * ((3.543 * Math.PI) / 540) / omniDriveFactor;
//
//        robotVectorByOdo = Math.atan2(robotYdisplacement, robotXdisplacement) * 180 / Math.PI;
//
//        robotVectorMag = Math.sqrt((robotXdisplacement * robotXdisplacement) + (robotYdisplacement * robotYdisplacement));
//
//        robotFieldAngle = (robotVectorByOdo + getAngle());
//
//        robotFieldPositionX = robotVectorMag * Math.cos(robotFieldAngle * Math.PI / 180);  //field position in inches
//        robotFieldPositionY = robotVectorMag * Math.sin(robotFieldAngle * Math.PI / 180);  //field position in inches
//
//        xPosition += robotFieldPositionX;
//        yPosition += robotFieldPositionY;
//
//
//        double [] displacement = {xPosition, yPosition, robotVectorByOdo};
//        return displacement;
//
//    }
//
//    boolean distanceTargetReached(double distanceToTarget){
//
//        boolean onDistanceTarget = false;
//
//        if (distanceToTarget < 5){
//            onDistanceTarget = true;
//        }
//
//
//        return  onDistanceTarget;
//    }
//
//    public void followCurve(ArrayList<CurvePoint> allPoints, double followAngle){
//
//        startTime = elapsedTime.seconds();
//        oldTime = startTime;
//
//        while (this.opmode.opModeIsActive() && !this.opmode.isStopRequested()) {
//
//            for (int i = 0; i < allPoints.size() - 1; i++) {
//                ComputerDebugging.sendLine(new FloatPoint(allPoints.get(i).x, allPoints.get(i).y),
//                        new FloatPoint(allPoints.get(i + 1).x, allPoints.get(i + 1).y));
//            }
//
//            double[] robotPositionXYV = findDisplacement(xPosition, yPosition, robotVectorByOdo);
//
//            CurvePoint followMe = getFollowPointPath(allPoints, new Point(robotPositionXYV[0], robotPositionXYV[1]),
//                    allPoints.get(0).followDistance);
//
//            ComputerDebugging.sendKeyPoint(new FloatPoint(followMe.x, followMe.y));
//
//            goToPositionPP(followMe.x, followMe.y, followMe.moveSpeed, followAngle, followMe.turnSpeed);
//        }
//    }
//
//    private CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, Point robotLocation, double followRadius){
//        CurvePoint followMe = new CurvePoint(pathPoints.get(0));
//
//        for(int i = 0; i < pathPoints.size() - 1; i++){
//            CurvePoint startLine = pathPoints.get(i);
//            CurvePoint endline = pathPoints.get(i + 1);
//
//            double[] robotPositionXYV = findDisplacement(xPosition, yPosition, robotVectorByOdo);
//
//            ArrayList<Point> intersections = MathFunctions.lineCircleIntersection(robotLocation, followRadius, startLine.toPoint(), endline.toPoint());
//
//
//            double closestAngle = 100000000;
//
//
//            for(Point thisIntersection : intersections){
//                double angle = Math.atan2(thisIntersection.y - robotPositionXYV[1], thisIntersection.x - robotPositionXYV[0]);
//                //Need to check which angle is the same as worldAngle_rad...currently using getAngle() converted to radians
//                double deltaAngle = Math.abs(MathFunctions.AngleWrap(angle - robotPositionXYV[2] * Math.PI / 180));
//
//                if(deltaAngle < closestAngle){
//                    closestAngle = deltaAngle;
//                    followMe.setPoint(thisIntersection);
//                }
//            }
//        }
//
//        if (debugFlag) {
//            RobotLog.d("FieldCentricInAutonTurn3Odo1 - followMe.x %f, followMe.y %f, followMe.moveSpeed %f, followMe.turnSpeed %f, followRadius %f, pointLength %f, slowDownTurnAmount %f, slowDownTurnRadians %f",
//                    followMe.x, followMe.y, followMe.moveSpeed, followMe.turnSpeed, followRadius, followMe.pointLength, followMe.slowDownTurnAmount, followMe.slowDownTurnRadians);
//        }
//
//        return followMe;
//    }
//
//
//    public void goToPositionPP(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed){
////        startTime = runtime;
//
////        distanceToTarget = 10;
//
////        while (this.opmode.opModeIsActive() && !this.opmode.isStopRequested()) {
//
//        //First calculate motor speeds for linear (x, y) motion
//
//        currentTime = elapsedTime.seconds();
//        loopTime = currentTime - oldTime;
//        oldTime = currentTime;
//        deltaTime = currentTime - startTime;
//
//        robotPositionXY = findDisplacement(xPosition, yPosition, robotVectorByOdo);
//
//        distanceToTarget = Math.hypot(x - robotPositionXY[0], y - robotPositionXY[1]);
//
//        double absoluteAngleToTarget = Math.atan2(y - robotPositionXY[1], x - robotPositionXY[0]) * 180 / Math.PI;
//
////            if (distanceToTarget < 5) {
////                robotTargetSpeed = 0;
////            } else {
////                robotTargetSpeed = movementSpeed;
////            }
//
//
//        robotTargetAngle = absoluteAngleToTarget;
//
//        robotAngleToTarget = MathFunctions.AngleWrapDeg((robotTargetAngle - 45) - getAngle());
//
//        xPower = Math.cos(robotAngleToTarget * 3.14 / 180) * movementSpeed;
//        yPower = Math.sin(robotAngleToTarget * 3.14 / 180) * movementSpeed;
//
//
//        double relativeTurnAngle = MathFunctions.AngleWrapDeg(robotAngleToTarget - 180 + preferredAngle);
//
//        if (distanceToTarget < 10) {
//            robotTurnSpeed = 0;
//        }else{
//            robotTurnSpeed = turnSpeed;
//        }
//
//        zPower = Range.clip(relativeTurnAngle / 30, -1, 1) * robotTurnSpeed;
//
//
//        //Second calculate motor speeds for angular (z) motion
//
////            robotCircumference = 2 * Math.PI * robotRadius;
////            robotWheelCircumference = 2 * Math.PI * robotWheelRadius;
////            wheelRotPerRobotRot = robotCircumference / robotWheelCircumference;
//
//        frontLeftMotorPower = -xPower + zPower;
//        rearRightMotorPower = xPower + zPower;
//        frontRightMotorPower = yPower + zPower;
//        rearLeftMotorPower = -yPower + zPower;
//
//        rearRightMotor.setPower(rearRightMotorPower);
//        frontLeftMotor.setPower(frontLeftMotorPower);
//        frontRightMotor.setPower(frontRightMotorPower);
//        rearLeftMotor.setPower(rearLeftMotorPower);
//
//
//
//
////            if (debugFlag) {
////                RobotLog.d("FieldCentricInAutonTurn3Odo1 - timeSinceStart %f, robotTargetAngle %f, xPower %f, yPower %f , zPower %f, frontLeftMotorPower %f, rearRightMotorPower %f , frontRightMotorPower %f, rearLeftMotorPower %f, frontLeftMotorTicks %f, rearRightMotorTicks %f , frontRightMotorTicks %f , rearLeftMotorTicks %f, xPosition %f, yPosition %f, robotRot %f, robotRotDisplacement %f, robotAngleToTarget %f, robotVectorByOdoF %f, robotVectorByOdoR %f, frontVectorMag %f, rearVectorMag %f",
////                        deltaTime, robotTargetAngle, xPower, yPower, zPower, frontLeftMotorPower, rearRightMotorPower, frontRightMotorPower, rearLeftMotorPower, lfDisplacement, rrDisplacement, rfDisplacement, lrDisplacement, xPosition, yPosition, robotRot, robotRotDisplacement, robotAngleToTarget, robotVectorByOdoF, robotVectorByOdoR, frontVectorMag, rearVectorMag);
////            }
//
//        if (debugFlag) {
//            RobotLog.d("FieldCentricInAutonTurn3Odo1 - runTime %f,robotPositionXY[0] %f, robotPositionXY[1] %f, distanceToTarget %f, relativeTurnAngle %f, robotVectorByOdo %f, robotVectorMag %f, robotFieldAngle %f, omniDriveAngle %f",
//                    deltaTime, robotPositionXY[0], robotPositionXY[1], distanceToTarget, relativeTurnAngle, robotVectorByOdo, robotVectorMag, robotFieldAngle, omniDriveAngle);
//        }
//
//
//
//
//
//
////        frontLeftMotor.setPower(0);
////        frontRightMotor.setPower(0);
////        rearLeftMotor.setPower(0);
////        rearRightMotor.setPower(0);
//
//    }
//
//
//}
//
