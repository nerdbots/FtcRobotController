/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

//Redo field centric Auton to be consistent with the right triangle diagram.

@Autonomous(name="OdometryTest", group="Linear Opmode")
//@Disabled
public class OdometryTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private BNO055IMU imu;
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor rearLeftMotor;
    private DcMotor rearRightMotor;
    Orientation angles;
    Acceleration gravity;

    Orientation lastAngles = new Orientation();

    double robotAngleToField = 0;
    double robotAngleToTarget = 0;

    double robotTargetSpeed = 0.5;
    double robotTargetAngle = 0;

    double xPower = 0;
    double yPower = 0;
    double zPower = 0;

    double frontLeftMotorPower = 0;
    double frontRightMotorPower = 0;
    double rearLeftMotorPower = 0;
    double rearRightMotorPower = 0;

    double robotRadius = 9.75; //robot radius in inches
    double robotCircumference;
    double robotWheelRadius = 3.5;  //robot wheel radius in inches
    double robotWheelCircumference;
    double wheelRotPerRobotRot;

//    double flCounts;
//    double frCounts;
//    double rlCounts;
//    double rrCounts;

    double xPosition = 0;
    double yPosition = 0;

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
    //    double robotFieldPositionXTicks = 0;
//    double robotFieldPositionYTicks = 0;
    double robotFieldPositionX = 0;
    double robotFieldPositionY = 0;
    double robotFieldAngle = 0;
    double displacementX = 0;
    double displacementY = 0;
    double [] robotPositionXY;
    double robotXdisplacement = 0;
    double robotYdisplacement = 0;

    double currentTime = 0;
    double deltaTime = 0;
    double startTime = 0;
    double oldTime = 0;
    double loopTime = 0;

    private ElapsedTime elapsedTime = new ElapsedTime();

    //private final String instanceName;
    private boolean debugFlag = true;

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        frontLeftMotor = hardwareMap.get(DcMotor.class, "Front_Left_Motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "Front_Right_Motor");
        rearLeftMotor = hardwareMap.get(DcMotor.class, "Rear_Left_Motor");
        rearRightMotor = hardwareMap.get(DcMotor.class, "Rear_Right_Motor");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);

        resetAngle();


        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);


        while (!isStopRequested() && !imu.isGyroCalibrated()) {

            sleep(50);
            idle();
        }

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        xPosition = 0;
        yPosition = 0;

        displacementX = 0;
        displacementY = 0;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        startTime = elapsedTime.seconds();

        while (opModeIsActive() && !isStopRequested()) {

            //First calculate motor speeds for linear (x, y) motion

            currentTime = elapsedTime.seconds();
            loopTime = currentTime - oldTime;
            oldTime = currentTime;
            deltaTime = currentTime - startTime;

            robotTargetAngle = 90;

            robotTargetSpeed = Math.min(((60 - yPosition) / 10), 1) * 1.0;

//            if (deltaTime < 3) {
//                robotTargetAngle = 135;
//                robotTargetSpeed = 0.5;
//                zPower = 0.0;
//            }
//            else if (deltaTime < 7.24) {
//                robotTargetAngle = 0;
//                robotTargetSpeed = 0.5;
//                zPower = 0.0;
//            }
//            else if (deltaTime < 10.24) {
//                robotTargetAngle = -135;
//                robotTargetSpeed = 0.5;
//                zPower = 0.0;
//            }
////        else if (deltaTime < 8){
////                robotTargetAngle = -45;
////                robotTargetSpeed = 0.5;
////                zPower = 0.0;
////            }
//            else {
//                robotTargetSpeed = 0.0;
//                zPower = 0.0;
//            }
            robotAngleToTarget = ((robotTargetAngle - 45) - getAngle());

            //robotTargetSpeed = 0.5;

            xPower = Math.cos(robotAngleToTarget * 3.14 / 180) * robotTargetSpeed;
            yPower = Math.sin(robotAngleToTarget * 3.14 / 180) * robotTargetSpeed;

            //Second calculate motor speeds for angular (z) motion

            robotCircumference = 2 * Math.PI * robotRadius;
            robotWheelCircumference = 2 * Math.PI * robotWheelRadius;
            wheelRotPerRobotRot = robotCircumference / robotWheelCircumference;

            //zPower = 0.2;

//        frontLeftMotorPower = zPower + xPower;
//        frontRightMotorPower = zPower + yPower;
//        rearLeftMotorPower = zPower + yPower;
//        rearRightMotorPower = -zPower - xPower;

            frontLeftMotorPower = -xPower + zPower;
            rearRightMotorPower = xPower + zPower;
            frontRightMotorPower = yPower + zPower;
            rearLeftMotorPower = -yPower + zPower;

            rearRightMotor.setPower(rearRightMotorPower);
            frontLeftMotor.setPower(frontLeftMotorPower);
            frontRightMotor.setPower(frontRightMotorPower);
            rearLeftMotor.setPower(rearLeftMotorPower);

//        flCounts = frontLeftMotor.getCurrentPosition();
//        frCounts = frontRightMotor.getCurrentPosition();
//        rlCounts = rearLeftMotor.getCurrentPosition();
//        rrCounts = rearRightMotor.getCurrentPosition();

            robotPositionXY = findDisplacement(displacementX, displacementY);

            xPosition += robotPositionXY[0];
            yPosition += robotPositionXY[1];

//            if (debugFlag) {
//                RobotLog.d("FieldCentricInAutonTurn3Odo1 - timeSinceStart %f, robotTargetAngle %f, xPower %f, yPower %f , zPower %f, frontLeftMotorPower %f, rearRightMotorPower %f , frontRightMotorPower %f, rearLeftMotorPower %f, frontLeftMotorTicks %f, rearRightMotorTicks %f , frontRightMotorTicks %f , rearLeftMotorTicks %f, xPosition %f, yPosition %f, robotRot %f, robotRotDisplacement %f, robotAngleToTarget %f, robotVectorByOdoF %f, robotVectorByOdoR %f, frontVectorMag %f, rearVectorMag %f",
//                        deltaTime, robotTargetAngle, xPower, yPower, zPower, frontLeftMotorPower, rearRightMotorPower, frontRightMotorPower, rearLeftMotorPower, lfDisplacement, rrDisplacement, rfDisplacement, lrDisplacement, xPosition, yPosition, robotRot, robotRotDisplacement, robotAngleToTarget, robotVectorByOdoF, robotVectorByOdoR, frontVectorMag, rearVectorMag);
//            }

            if (debugFlag) {
                RobotLog.d("FieldCentricInAutonTurn3Odo1 - timeSinceStart %f, robotTargetAngle %f, lfDisplacementNew %f, lfDispNoRot %f, rrDisplacementNew %f, rrDispNoRot %f, rfDisplacementNew %f, rfDispNoRot %f, lrDisplacementNew %f, lrDispNoRot %f, xPosition %f, yPosition %f, robotRot %f, robotRotDisplacement %f, robotAngleToTarget %f, robotVectorByOdo %f, robotVectorMag %f, robotFieldAngle %f, robotFieldPositionX %f, robotFieldPositionY %f, robotXdisplacement %f, robotYdisplacement %f, omniDriveAngle %f, omniDriveFactor %f",
                        deltaTime, robotTargetAngle, lfDisplacementNew, lfDispNoRot, rrDisplacementNew, rrDispNoRot, rfDisplacementNew, rfDispNoRot, lrDisplacementNew, lrDispNoRot, xPosition, yPosition, robotRot, robotRotDisplacement, robotAngleToTarget, robotVectorByOdo, robotVectorMag, robotFieldAngle, robotFieldPositionX, robotFieldPositionY, robotXdisplacement, robotYdisplacement, omniDriveAngle, omniDriveFactor);
            }

//            if (debugFlag) {
//                RobotLog.d("FieldCentricInAutonTurn3Odo1 FindDisplacement - robotRotNew %f, robotRot %f, robotRotOld %f, robotRotDisplacement %f, rfDisplacement %f, lfDisplacement %f, lrDisplacement %f, rrDisplacement %f, lfDispNoRot %f, rrDispNoRot %f, rfDispNoRot %f, lrDispNoRot %f, robotVectorByOdoF %f, robotVectorByOdoR %f, frontVectorMag %f, rearVectorMag %f, frontDisplacementX %f, frontDisplacementY %f, rearDisplacementX %f, rearDisplacementY %f",
//                        robotRotNew, robotRot, robotRotOld, robotRotDisplacement, rfDisplacement, lfDisplacement, lrDisplacement, rrDisplacement, lfDispNoRot, rrDispNoRot, rfDispNoRot, lrDispNoRot, robotVectorByOdoF, robotVectorByOdoR, frontVectorMag, rearVectorMag, frontDisplacementX, frontDisplacementY, rearDisplacementX, rearDisplacementY);
//            }
//
//            final String funcName = "fieldDisplacementXY";
//            if (debugFlag) {
//                RobotLog.d("FieldCentricInAutonTurn3Odo1b - field displacement : %s |displacmentX | displacementY", funcName);
//                RobotLog.d("FieldCentricInAutonTurn3Odo1b - field displacement : %s |%f|%f", funcName,robotPositionXY[0],robotPositionXY[1]);
//            }

//            if (debugFlag)
//                RobotLog.d ("FieldCentricInAutonTurn3Odo1 - xPower = %f, yPower = %f , zPower %f", xPower, yPower, zPower);
//            if (debugFlag)
//                RobotLog.d ("FieldCentricInAutonTurn3Odo1 - frontLeftMotorPower = %f, rearRightMotorPower = %f , frontRightMotorPower %f , rearLeftMotorPower %f", frontLeftMotorPower, rearRightMotorPower, frontRightMotorPower, rearLeftMotorPower);
//            if (debugFlag)
//                RobotLog.d ("FieldCentricInAutonTurn3Odo1 - frontLeftMotorTicks = %f, rearRightMotorTicks = %f , frontRightMotorTicks %f , rearLeftMotorTicks %f", flCounts, rrCounts, frCounts, rlCounts);
//            if (debugFlag)
//                RobotLog.d ("FieldCentricInAutonTurn3Odo1 - xDisplacement = %f, yDisplacement = %f", xDisplacement, yDisplacement);
//            if (debugFlag)
//                RobotLog.d ("FieldCentricInAutonTurn3Odo1 - Time = %f, Time Since Start = %f , Loop Time %f", currentTime, deltaTime, loopTime);
//            if (debugFlag)
//                RobotLog.d ("FieldCentricInAutonTurn3Odo1 - Robot Target Angle = %f, Robot Angle to Target = %f , Target Speed %f", robotTargetAngle, robotAngleToTarget, robotTargetSpeed);



            //robotTargetAngle = robotTargetAngle + 1;

            telemetry.addData("xDiplacement", robotPositionXY[0]);
            telemetry.addData("yDisplacement", robotPositionXY[1]);
            telemetry.addData("LF", lfDisplacementNew);
            telemetry.addData("RF", rfDisplacementNew);
            telemetry.addData("LR", lrDisplacementNew);
            telemetry.addData("RR", rrDisplacementNew);
            telemetry.update();

        }

        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);

        telemetry.addData("xDiplacement", robotPositionXY[0]);
        telemetry.addData("yDisplacement", robotPositionXY[1]);
        telemetry.addData("LF", lfDisplacementNew);
        telemetry.addData("RF", rfDisplacementNew);
        telemetry.addData("LR", lrDisplacementNew);
        telemetry.addData("RR", rrDisplacementNew);
        telemetry.update();

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

    public double [] findDisplacement(double displacementX, double displacementY){

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

        omniDriveAngle = robotAngleToTarget;

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
        robotXdisplacement = ((-rfDispNoRot - lfDispNoRot + rrDispNoRot + lrDispNoRot) / 4) * ((3.543 * Math.PI) / 560) / omniDriveFactor;
        //calculate Y displacement, and convert ticks to inches (3.543 = wheel diameter inches, 560 = ticks per wheel rot), and account for robot angle
        robotYdisplacement = ((rfDispNoRot - lfDispNoRot + rrDispNoRot - lrDispNoRot) / 4) * ((3.543 * Math.PI) / 560) / omniDriveFactor;

        robotVectorByOdo = Math.atan2(robotYdisplacement, robotXdisplacement) * 180 / Math.PI;

        robotVectorMag = Math.sqrt((robotXdisplacement * robotXdisplacement) + (robotYdisplacement * robotYdisplacement));

        robotFieldAngle = (robotVectorByOdo + getAngle());

        robotFieldPositionX = robotVectorMag * Math.cos(robotFieldAngle * Math.PI / 180);  //field position in inches
        robotFieldPositionY = robotVectorMag * Math.sin(robotFieldAngle * Math.PI / 180);  //field position in inches

//        robotFieldPositionX = robotFieldPositionXTicks * 0.02060; //convert ticks to inches
//        robotFieldPositionY = robotFieldPositionYTicks * 0.02060; //convert ticks to inches

        double [] displacement = {robotFieldPositionX, robotFieldPositionY};
        return displacement;

    }

    //Function to find out the Robot travel distance in Y direction.


//    double findYDisplacement(){
//
//        robotRotNew = getAngle();
//        robotRot = robotRotNew - robotRotOld;
//        robotRotOld = robotRotNew;
//
//        robotRotDisplacement = robotRot * (19.625/2) *(560 / (Math.PI * 3.543));
//
//        rfDisplacement = frontRightMotor.getCurrentPosition();
//        lfDisplacement = frontLeftMotor.getCurrentPosition();
//        lrDisplacement = rearLeftMotor.getCurrentPosition();
//        rrDisplacement = rearRightMotor.getCurrentPosition();
//
//        robotVectorByOdoF = Math.atan2((rfDisplacement - robotRotDisplacement), (lfDisplacement - robotRotDisplacement));
//        robotVectorByOdoR = Math.atan2((lrDisplacement - robotRotDisplacement), (rrDisplacement - robotRotDisplacement));
//
//        frontVectorMag = Math.sqrt((lfDisplacement * lfDisplacement) + (rfDisplacement * rfDisplacement));
//        rearVectorMag = Math.sqrt((lrDisplacement * lrDisplacement) + (rrDisplacement * rrDisplacement));
//
//        frontDisplacementX = frontVectorMag * Math.cos(robotVectorByOdoF);
//        frontDisplacementY = frontVectorMag * Math.sin(robotVectorByOdoF);
//
//        rearDisplacementX = rearVectorMag * Math.cos(robotVectorByOdoR);
//        rearDisplacementY = rearVectorMag * Math.sin(robotVectorByOdoR);
//
//        displacementX = (frontDisplacementX + rearDisplacementX) / 2;
//        displacementY = (frontDisplacementY + rearDisplacementY) / 2;
//
//        return displacementY;
//
//    }
}

