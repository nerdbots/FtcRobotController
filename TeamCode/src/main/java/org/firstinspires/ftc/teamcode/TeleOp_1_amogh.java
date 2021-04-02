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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.sql.Time;

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
@Disabled
@TeleOp(name="Teleop_amogh", group="Final")
public class TeleOp_1_amogh extends LinearOpMode {

    NERDTFObjectDetector nerdtfObjectDetector ;
    NERDShooterClass nerdShooterClass;
    public Recognition recognition;

    private DcMotor wobbleMotor;
    Servo wobbleServo;

    private ElapsedTime Timer = new ElapsedTime();


    boolean pressedOnce = false;


    private BNO055IMU imu;
    private DcMotor frontRightMotor;
    private DcMotor rearRightMotor;
    private DcMotor frontLeftMotor;
    private DcMotor rearLeftMotor;
    private DcMotorEx shooter;
    private DcMotor intake;
    private  Servo indexingServo;


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


    private double ZkP = 0.013; //0.011
    private double ZkI = 0.000; //0.000
    private double ZkD = 0.0013;//0.00145


    double kP = 0.01;
    double kI = 0;
    double kD = 0;
    boolean targetSet =false;
    double maxAlignSpeed=0.5;

    private double ZTar = 0;

    private double MaxSpeedZ = 1.0;

    private int shooterveloc = -2000;


    @Override
    public void runOpMode() {

        nerdtfObjectDetector = new NERDTFObjectDetector(this, "BlueGoalOnly.tflite", "BlueGoal", 610, 5);
        nerdShooterClass = new NERDShooterClass(this);
        // nerdPIDCalculator = new NerdPIDCalculator("goalTarget", kP, kI, kD);
        nerdtfObjectDetector.initialize();


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        rearLeftMotor = hardwareMap.get(DcMotor.class, "Rear_Left_Motor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "Front_Left_Motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "Front_Right_Motor");
        rearRightMotor = hardwareMap.get(DcMotor.class, "Rear_Right_Motor");
        wobbleMotor = hardwareMap.get(DcMotor.class, "Left");
        wobbleServo = hardwareMap.get(Servo.class, "wobble_Goal_Servo");
        shooter = hardwareMap.get(DcMotorEx.class, "Front");
        intake = hardwareMap.get(DcMotor.class, "Right");
        indexingServo = hardwareMap.get(Servo.class, "indexingServo");


        //  globalAngle = 0;/imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;


//        expansion_Hub_1 = hardwareMap.get(Blinker.class, "Expansion Hub 2");


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);


        resetAngle();

        nerdShooterClass.initialize();



        /*
        rearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(500);

        rearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

*/

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)


        double positionPitch = 0.52;  // (MAX_POS - MIN_POS) / 2;
        double positionAngle = 0.15;  //(MAX_POS - MIN_POS) / 2; 0.25
        double tapeSpeed = 0.0;


        double LMP = 0;
        double RMP = 0;
        double FMP = 0;
        double BMP = 0;

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

        boolean isShooterMode = false;

        boolean pressedOnce_shooter = false;


        shooter.setVelocityPIDFCoefficients(200, 0.1, 0, 16);


        //int targetR = -80;
        //int targetF = 60;


        if (!imu.isGyroCalibrated()) {
            telemetry.addData("Gyro", "Not Initialized");
            telemetry.update();

            sleep(200);

            telemetry.addData("Sike!", "Yeah, go ahead and press start");
            telemetry.update();
        }


        waitForStart();


        //   resetAngle();

        ZPIDTime.reset();

        Timer.reset();

        Timer.reset();

        wobbleTime.reset();


//        if(opModeIsActive()) {
//            Timer.reset();
//            while(Timer.seconds() < 0.25) {
//                wobbleMotor.setPower(-0.55);
//            }
//            wobbleMotor.setPower(0);
//
//            wobbleServo.setPosition(0);
//        }

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            isShooterMode = gamepad1.left_bumper;

            if (gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0) {
                joyX = gamepad1.left_stick_x;
                joyY = gamepad1.left_stick_y;
            }

            if (gamepad1.dpad_up) {
                joyX = 0;
                joyY = -1;
            } else if (gamepad1.dpad_down) {
                joyX = 0;
                joyY = 1;
            } else if (gamepad1.dpad_left) {
                joyX = -1;
                joyY = 0;
            } else if (gamepad1.dpad_right) {
                joyX = 1;
                joyY = 0;
            }


            if (gamepad1.y) {
                resetAngle();
            }


            if (gamepad1.b) {
                BNO055IMU.Parameters parametersb = new BNO055IMU.Parameters();

                parametersb.mode = BNO055IMU.SensorMode.IMU;
                parametersb.angleUnit = BNO055IMU.AngleUnit.DEGREES;
                parametersb.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
                parametersb.loggingEnabled = false;
                parametersb.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
                parametersb.loggingTag = "IMU";
                parametersb.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

                imu.initialize(parametersb);

            }


            zMag = (joyX * joyX) + (joyY * joyY);


            if (Math.sqrt(zMag) > 0.5) {
                ZTar = Math.atan2(-joyX, -joyY) * 180 / 3.14159;

            }


            if (isShooterMode) {
                multZ = 0.3;
                mult = 0.5;
            } else {
                multZ = 0.6;
                mult = 1;
            }


            CA = (Math.atan2(gamepad1.right_stick_y, -gamepad1.right_stick_x) * 180 / 3.14) + 45;

            RSA = (CA - getAngle()) * 3.14 / 180;

            Mag = Math.sqrt((gamepad1.right_stick_y * gamepad1.right_stick_y) + (gamepad1.right_stick_x * gamepad1.right_stick_x));

            FX = -Math.sin(RSA) * Mag;
            FY = -Math.cos(RSA) * Mag;

            if (isShooterMode) {
                LMP = -gamepad1.left_stick_x + FX;
                RMP = -gamepad1.left_stick_x - FX;
                FMP = -gamepad1.left_stick_x + FY;
                BMP = -gamepad1.left_stick_x - FY;

            } else {
                LMP = (ZSpeed * multZ) + FX; //multZ will only affect Z. This is because if joypad Z is zero then Z is zero.
                RMP = (ZSpeed * multZ) - FX;
                FMP = (ZSpeed * multZ) + FY;
                BMP = (ZSpeed * multZ) - FY;
            }

            frontLeftMotor.setPower(RMP * mult);
            rearRightMotor.setPower(LMP * mult);

            rearLeftMotor.setPower(FMP * mult);
            frontRightMotor.setPower(BMP * mult);


            PIDArm(getAngle(), ZTar, ZkP, ZkI, ZkD, 67);//CAN BE ANYTHING BUT 0 OR 1

            //up

            if (gamepad2.a) {
                wobbleServo.setPosition(0.7);
            }
            //down
            if (gamepad2.b) {
                wobbleServo.setPosition(0);

            }




//            if(gamepad2.a && pressedOnce == false) {
//                pressedOnce = true;
//
//                wobbleServo.setPosition(0.7);
//
//                sleep(500);
//
//                Timer.reset();
//                while(Timer.seconds() < 0.88) {
//                    wobbleMotor.setPower(0.9);
//                }
//                wobbleMotor.setPower(0.1);
//
//            }
//            //down
//            if(gamepad2.a && pressedOnce == true) {
//                pressedOnce = false;
//
//
//                sleep(500);
//                wobbleServo.setPosition(0);
//
//                Timer.reset();
//                while(Timer.seconds() < 0.17) {
//                    wobbleMotor.setPower(-0.55);
//                }
//
//
//                wobbleMotor.setPower(0);
//
//
//                //          sleep(500);
//
//            }


            if (gamepad2.dpad_left) {
                wobbleMotor.setPower(0);
            }

            if (gamepad2.dpad_up) {
                wobbleMotor.setPower(1);
                wobbleTime.reset();
            }
            if (gamepad2.dpad_down) {
                wobbleMotor.setPower(-1);
                wobbleTime.reset();
            }
            if (wobbleTime.seconds() < 50 && !gamepad2.dpad_up && !gamepad2.dpad_down) {
                wobbleMotor.setPower(0);
            }


            if (gamepad2.left_trigger > 0.75) {
                intake.setPower(1);
            } else if (gamepad2.right_trigger > 0.75) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }

            if (gamepad2.left_bumper && pressedOnce_shooter == false) {
                pressedOnce_shooter = true;
                shooter.setPower(-.90);
                sleep(200);

            }
            if (gamepad2.left_bumper && pressedOnce_shooter == true) {
                pressedOnce_shooter = false;

                shooter.setPower(0);

                sleep(200);

            }

            if (gamepad2.right_bumper) {
                indexingServo.setPosition(0.45);
                sleep(500);
                indexingServo.setPosition(1);
            }
            //high goal
            if (gamepad2.x) {
                shooter.setVelocity(-2000);


                recognition = nerdtfObjectDetector.detect("BlueGoal", true);
                if (recognition != null) {

                    telemetry.addData("Angle to Object", recognition.estimateAngleToObject(AngleUnit.DEGREES));
                    telemetry.addData("Image Center", nerdtfObjectDetector.findImageCenter(recognition));
                    telemetry.addData("Object Center", nerdtfObjectDetector.findObjectCenter(recognition));
                    telemetry.addData("Encoder Value Front Left", frontLeftMotor.getCurrentPosition());
                    telemetry.addData("Encoder Value Front Right", frontRightMotor.getCurrentPosition());
                    telemetry.addData("Encoder Value Rear Left", rearLeftMotor.getCurrentPosition());
                    telemetry.addData("Encoder Value Rear Right", rearRightMotor.getCurrentPosition());
                    telemetry.update();
//                    sleep(5000);

                    boolean isTargetAligned = false;
                    int count = 0;
                    while ((recognition != null) && !isTargetAligned) {
                        if (recognition != null) {
                            if (nerdtfObjectDetector.findObjectCenter(recognition) <= 360) {
                                telemetry.addData("Uh Oh:", "You're too far Right. move Left");
                                telemetry.update();
                                frontLeftMotor.setPower(maxAlignSpeed);
                                rearLeftMotor.setPower(maxAlignSpeed);
                                frontRightMotor.setPower(maxAlignSpeed);
                                rearRightMotor.setPower(maxAlignSpeed);
                            } else if (nerdtfObjectDetector.findObjectCenter(recognition) >= 380) {
                                telemetry.addData("Uh Oh:", "You're too far Left. move Right");
                                telemetry.update();
                                frontRightMotor.setPower(-maxAlignSpeed);
                                rearRightMotor.setPower(-maxAlignSpeed);
                                frontLeftMotor.setPower(-maxAlignSpeed);
                                rearLeftMotor.setPower(-maxAlignSpeed);
                            } else {

                                telemetry.addData("You're Good to go:", "You are within the 20 pixel threshold");
                                telemetry.addData("Encoder Value Front Left", frontLeftMotor.getCurrentPosition());
                                telemetry.addData("Encoder Value Front Right", frontRightMotor.getCurrentPosition());
                                telemetry.addData("Encoder Value Rear Left", rearLeftMotor.getCurrentPosition());
                                telemetry.addData("Encoder Value Rear Right", rearRightMotor.getCurrentPosition());
                                telemetry.update();
////                                sleep(2000);
//                                while(!isTargetAligned) {
////                                    if ((Math.abs(shooter.getVelocity()))- Math.abs(shooterveloc)) < 20)) {
//                                    if((Math.abs(shooter.getVelocity()) - Math.abs(shooterveloc)) < 20) {
//                                        nerdShooterClass.indexRingsOnce();
//                                        isTargetAligned = true;
//                                    }
//                                }

                                while (count < 3) {

                                    if(Math.abs(Math.abs(shooter.getVelocity()) - Math.abs(shooterveloc)) < 20) {
                                        nerdShooterClass.indexRingsOnce();
                                        count++;

                                    }

                                }


                            }
                            frontRightMotor.setPower(0);
                            rearRightMotor.setPower(0);
                            frontLeftMotor.setPower(0);
                            rearLeftMotor.setPower(0);
                        } else {
                            telemetry.addData("too bad", "it found nothing");
                            telemetry.update();
                            sleep(1000);
                        }
                        recognition = nerdtfObjectDetector.detect("BlueGoal", true);

                    }

//                    telemetry.addData("count:",count);
//                    telemetry.update();


                }
                shooter.setVelocity(0);
            }

            //power shots
            if (gamepad2.y) {
                recognition = nerdtfObjectDetector.detect("BlueGoal", true);
                if (recognition != null) {

                    telemetry.addData("Angle to Object", recognition.estimateAngleToObject(AngleUnit.DEGREES));
                    telemetry.addData("Image Center", nerdtfObjectDetector.findImageCenter(recognition));
                    telemetry.addData("Object Center", nerdtfObjectDetector.findObjectCenter(recognition));
                    telemetry.addData("Object Right", recognition.getRight());
                    telemetry.addData("Encoder Value Front Left", frontLeftMotor.getCurrentPosition());
                    telemetry.addData("Encoder Value Front Right", frontRightMotor.getCurrentPosition());
                    telemetry.addData("Encoder Value Rear Left", rearLeftMotor.getCurrentPosition());
                    telemetry.addData("Encoder Value Rear Right", rearRightMotor.getCurrentPosition());
                    telemetry.update();
                    sleep(2000);

                    sleep(2000);

                    boolean isTargetAligned = false;
                    boolean powerShotsShot = false;
                    int count = 0;
                    double targetLockedPosition;
                    double powerShotAimPosition = 370;
                    while (!(recognition == null) && !isTargetAligned && (count < 3)) {
                        if (recognition != null) {
                            if (recognition.getRight() <= powerShotAimPosition) {
                                telemetry.addData("Uh Oh:", "You're too far Right. move Left");
                                telemetry.update();
                                frontLeftMotor.setPower(maxAlignSpeed);
                                rearLeftMotor.setPower(maxAlignSpeed);
                                frontRightMotor.setPower(maxAlignSpeed);
                                rearRightMotor.setPower(maxAlignSpeed);
                            } else if (recognition.getRight() >= powerShotAimPosition + 30) {
                                telemetry.addData("Uh Oh:", "You're too far Lft. move Right");
                                telemetry.update();
                                frontRightMotor.setPower(-maxAlignSpeed);
                                rearRightMotor.setPower(-maxAlignSpeed);
                                frontLeftMotor.setPower(-maxAlignSpeed);
                                rearLeftMotor.setPower(-maxAlignSpeed);
                            } else {

                                targetLockedPosition = recognition.getRight();

                                telemetry.addData("TargetLockedPos", targetLockedPosition);
                                telemetry.addData("Encoder Value Front Left", frontLeftMotor.getCurrentPosition());
                                telemetry.addData("Encoder Value Front Right", frontRightMotor.getCurrentPosition());
                                telemetry.addData("Encoder Value Rear Left", rearLeftMotor.getCurrentPosition());
                                telemetry.addData("Encoder Value Rear Right", rearRightMotor.getCurrentPosition());
                                telemetry.update();
                                sleep(2000);
                                nerdShooterClass.indexRingsOnce();
                                powerShotAimPosition = powerShotAimPosition - 30;
                                count++;

                                if (count == 3) isTargetAligned = true;
                            }
                            frontRightMotor.setPower(0);
                            rearRightMotor.setPower(0);
                            frontLeftMotor.setPower(0);
                            rearLeftMotor.setPower(0);
                        } else {
                            telemetry.addData("too bad", "it found nothing");
                            telemetry.update();
                            sleep(1000);
                        }
                        recognition = nerdtfObjectDetector.detect("BlueGoal", true);

                    }

                    telemetry.addData("count:", count);
                    telemetry.update();


                }
            }


            //add telemetry


            telemetry.addData("IsPressed", pressedOnce);
            telemetry.addData("X", FX);
            telemetry.addData("Y", FY);
            telemetry.addData("CA", CA);
            telemetry.addData("RSA", RSA);
            telemetry.addData("RA", getAngle());

            telemetry.addData("zMag", zMag);
            telemetry.addData("ZTar", ZTar);

            telemetry.addData("FREV", frontRightMotor.getCurrentPosition());
            telemetry.addData("FLEV", frontLeftMotor.getCurrentPosition());
            telemetry.addData("RREV", rearRightMotor.getCurrentPosition());
            telemetry.addData("RLEV", rearLeftMotor.getCurrentPosition());


            telemetry.addData("Status", "Running");
            //   telemetry.update();
            //  }

        }
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


        ZSpeed = speed;
        ZPrevError = PrevError;
        ZTotalError = TotalError;

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
}

