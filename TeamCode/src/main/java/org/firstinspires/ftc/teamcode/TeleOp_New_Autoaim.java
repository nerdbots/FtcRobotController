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
//@Disabled
@TeleOp(name="Teleop", group="Final")
public class TeleOp_New_Autoaim extends LinearOpMode {


    boolean useVisionShoot = false;

    boolean visionTelemetry = false;

    double turnAngleVision = 0;

    double angleOffsetVision = -5; //1.3

    NERDTFObjectDetector nerdtfObjectDetector ;
    NERDShooterClass_TeleOp nerdShooterClass;
    public Recognition recognition;

    NerdVelocityFollowing_Teleop NerdVelocityFollowing = new NerdVelocityFollowing_Teleop();

    private DcMotor wobbleMotor;
    Servo wobbleServo;

    private ElapsedTime Timer = new ElapsedTime();


    boolean pressedOnce = false;

    boolean isDetected = false;


    private BNO055IMU imu;
    private DcMotor frontRightMotor;
    private DcMotor rearRightMotor;
    private DcMotor frontLeftMotor;
    private DcMotor rearLeftMotor;
    private DcMotorEx shooter;
    private DcMotor intake;
    private Servo indexingServo;
    private Servo kickerServo;

    private ElapsedTime elapsedTime = new ElapsedTime();
    private  ElapsedTime kickerTime = new ElapsedTime();


    private double shooterveloc = -1425;

    Orientation angles;
    Acceleration gravity;

    Orientation lastAngles = new Orientation();

    double globalAngle = 0.0;


    private ElapsedTime ZPIDTime = new ElapsedTime();

    private ElapsedTime PIDTime = new ElapsedTime();

    private ElapsedTime wobbleTime = new ElapsedTime();

    private double ZPrevError = 0;


    private double ZTotalError = 0;

    private double ZTotalError_Vision = 0;


    private double ZSpeed = 0;


    private double ZDerror = 0;


    private double ZkP = 0.014; //0.014
    private double ZkI = 0.005; //0.000
    private double ZkD = 0.0013;//0.0013


    double kP = 0.01;
    double kI = 0;
    double kD = 0;
    boolean targetSet =false;
    double maxAlignSpeed=0.5;

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

    double count = 0;

    double ZTarVision = 0;

    boolean sendMotorCommands = true;

    boolean visionSuccessful = false;


    double visionAngle = 0;



    @Override
    public void runOpMode() {

        nerdtfObjectDetector= new NERDTFObjectDetector(this, "BlueGoal.tflite", "BlueGoal", 570,  24);
        nerdShooterClass = new NERDShooterClass_TeleOp(this);
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
        kickerServo = hardwareMap.get(Servo.class, "Kicker_Servo");


        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);





        //  globalAngle = 0;/imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;


//        expansion_Hub_1 = hardwareMap.get(Blinker.class, "Expansion Hub 2");









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

     //   nerdShooterClass.initialize();

        double frontLeftMotorPower = 0;
        double frontRightMotorPower = 0;
        double rearLeftMotorPower = 0;
        double rearRightMotorPower = 0;
        double maxPowerEndPP = 0;




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


        double fieldCentricMororPowerFL = 0;
        double fieldCentricMororPowerFR = 0;
        double fieldCentricMororPowerRL = 0;
        double fieldCentricMororPowerRR = 0;



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

        boolean pressedOnce_shooter_powerShots = false;

        boolean pressedOnce_intake = false;
        boolean pressedOnce_outtake = false;

        double visionTarget = 0;

        boolean pressed_Once_IntakeKicker2 = true;

        boolean pressedOnceIntakeKicker = false;
        boolean pressedOncceIntakeKicker3 = false;





        //int targetR = -80;
        //int targetF = 60;


//        if (!imu.isGyroCalibrated()) {
//            telemetry.addData("Gyro", "Not Initialized");
//            telemetry.update();
//
//            sleep(200);
//
//            telemetry.addData("Sike!", "Yeah, go ahead and press start");
//            telemetry.update();
//        }


        waitForStart();

        kickerServo.setPosition(-1);
        indexingServo.setPosition(0.65);

        shooter.setVelocityPIDFCoefficients(200, 10, 0, 16);

        boolean pressedOnceKicker = true;

        boolean pressed_Once_Vision = true;

        boolean pressed_Once_Vision2 = true;



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

            //Slowmode
            isShooterMode = gamepad1.left_bumper;

            //Get Joystick commands for "Smash Mouth" turning
            if (gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0 ) {
                joyX = gamepad1.left_stick_x;
                joyY = gamepad1.left_stick_y;
            }

            //Quick turn using dpad
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




            //Quick reset gyro button
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


            //Calculation for "Smash Mouth" turning
            zMag = (joyX * joyX) + (joyY * joyY);


            //Turns off manual input if autoshooting using vision, does final calculations for turning

            if (Math.sqrt(zMag) > 0.5)
                ZTar = Math.atan2(-joyX, -joyY) * 180 / 3.14159;




            //Turns on or off Slowmode
//            if (isShooterMode) {
//                multZ = 0.6;
//                mult = 0.5;
//            } else {
                multZ = 0.6;
                mult = 1;
//            }


            //Drive calculatiokns for field-centric
            CA = (Math.atan2(gamepad1.right_stick_y, -gamepad1.right_stick_x) * 180 / 3.14) + 45;

            RSA = (CA - getAngle()) * 3.14 / 180;

            Mag = Math.sqrt((gamepad1.right_stick_y * gamepad1.right_stick_y) + (gamepad1.right_stick_x * gamepad1.right_stick_x));

            FX = -Math.sin(RSA) * Mag;
            FY = -Math.cos(RSA) * Mag;

            //Direct tank turning control if slowmode is on
            if(isShooterMode){
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

            //Outputs field-centric motor commands so Velocity Following
            fieldCentricMororPowerFL = RMP * mult;
            fieldCentricMororPowerRR = LMP * mult;

            fieldCentricMororPowerRL = FMP * mult;
            fieldCentricMororPowerFR = BMP * mult;

            VelocityCommands(fieldCentricMororPowerFL, fieldCentricMororPowerRR, fieldCentricMororPowerRL, fieldCentricMororPowerFR);

            if(useVisionShoot && visionSuccessful) {
//                PIDArm(-visionAngle, ZTarVision, ZkP, 0.0001, ZkD, 67);//CAN BE ANYTHING BUT 0 OR 1
                PIDArm(getAngle(), -ZTarVision, 0.019, 0.005, 0.002, 67);//CAN BE ANYTHING BUT 0 OR 1         0.023, 0.01, 0.0012
                telemetry.addData("target angle", -ZTarVision);
                telemetry.addData("Gyro Angle", getAngle());
                telemetry.addData("vision angle", -visionAngle);
                telemetry.update();

            } else {
                PIDArm(getAngle(), ZTar, ZkP, ZkI, ZkD, 67);//CAN BE ANYTHING BUT 0 OR 1
                telemetry.addData("target angle", ZTar);
                telemetry.addData("Gyro Angle", getAngle());
                telemetry.update();
            }

            //todo: THE THING IS HERE

            //up

            if(gamepad2.a) {
//                wobbleServo.setPosition(0.7);
            wobbleServo.setPosition(1);
            }
            //down
            if(gamepad2.b) {
//                wobbleServo.setPosition(0);

                wobbleServo.setPosition(-1);
            }



            if(gamepad1.a) {
                NerdVelocityFollowing_Teleop.resetI();
            }


            if(gamepad2.dpad_left) {
                wobbleMotor.setPower(0);
            }

            if(gamepad2.dpad_up) {
                wobbleMotor.setPower(1);
                wobbleTime.reset();
            }
            if(gamepad2.dpad_down) {
                wobbleMotor.setPower(-1);
                wobbleTime.reset();
            }
            if(wobbleTime.seconds() < 50 && !gamepad2.dpad_up && !gamepad2.dpad_down) {
                wobbleMotor.setPower(0);
            }


            if(pressedOnceIntakeKicker) {intake.setPower(0);}
            else if(((gamepad2.left_trigger > 0.75 || gamepad1.left_trigger > 0.75) && pressedOnce_intake == false) || (pressed_Once_IntakeKicker2 && pressedOnce_intake == false)) {
                intake.setPower(1);
                pressedOnce_intake = true;
                sleep(200);
            } else if((gamepad2.right_trigger > 0.75 || gamepad1.right_trigger > 0.75)  && pressedOnce_outtake == false) {
                intake.setPower(-1);
                pressedOnce_outtake = true;
                sleep(200);
            } else if(((gamepad2.right_trigger > 0.75 || gamepad1.right_trigger > 0.75) || (gamepad2.left_trigger > 0.75 || gamepad1.left_trigger > 0.75)) && (pressedOnce_intake == true || pressedOnce_outtake)) {
                intake.setPower(0);
                pressedOnce_intake = false;
                pressedOnce_outtake = false;
                sleep(200);
            }

            if((gamepad2.left_bumper || gamepad1.left_bumper) && pressedOnce_shooter == false) {
                pressedOnce_shooter = true;
                shooter.setVelocity(shooterveloc);
                sleep(200);

            } if((gamepad2.left_bumper || gamepad1.left_bumper) && pressedOnce_shooter == true) {
                pressedOnce_shooter = false;

                shooter.setVelocity(0);

                sleep(200);
            }

            if(gamepad2.right_stick_button && pressedOnce_shooter_powerShots == false) {
                pressedOnce_shooter_powerShots = true;
                shooter.setVelocity(-1325);
                sleep(200);

            } if(gamepad2.right_stick_button && pressedOnce_shooter_powerShots == true) {
                pressedOnce_shooter_powerShots = false;

                shooter.setVelocity(0);

                sleep(200);
            }


            if(gamepad2.dpad_right) {
                shooter.setPower(1);
                sleep(1000);
                shooter.setPower(0);
            }

            if((gamepad2.right_bumper || gamepad1.right_bumper) && (Math.abs(shooter.getVelocity()) > Math.abs(-500))) {
                indexingServo.setPosition(0.65);
                kickerServo.setPosition(1);
                sleep(500);
                kickerServo.setPosition(-1);
                sleep(50);
                indexingServo.setPosition(0.1);
                sleep(500);
                indexingServo.setPosition(0.65);
            }

            if(gamepad2.y) {
                if(pressedOnceKicker) {
                    pressed_Once_IntakeKicker2 = true;
                    pressedOnceIntakeKicker = true;
                    intake.setPower(0);
                    kickerServo.setPosition(1);
                    kickerTime.reset();
                    pressedOnceKicker = false;
                } if((!(kickerTime.seconds() <= 0.5))) {
                    kickerServo.setPosition(-1);
                }
                if(pressedOnce_intake) {
                    pressedOncceIntakeKicker3 = true;
                } else {
                    pressedOncceIntakeKicker3 = false;
                }
            } else {
                pressedOnceKicker = true;
                pressedOnceIntakeKicker = false;
                pressed_Once_IntakeKicker2 = false;
            }


            if(gamepad1.x) {
                if(pressed_Once_Vision == true) {
                    visionAngle = getAngleOnce();
                    ZTarVision = getAngle() + visionAngle;
                }

                pressed_Once_Vision = false;
            } else {
                useVisionShoot = false;
                sendMotorCommands = true;
                visionSuccessful = false;
                pressed_Once_Vision = true;
                ZTotalError_Vision = 0;
                ZTarVision = 0;

              //  kickerServo.setPosition(-1);
            }

//            if(gamepad1.x && pressed_Once_Vision2 == false) {
//                pressed_Once_Vision2 = true;
//                if(pressed_Once_Vision == true) {
//                    visionAngle = getAngleOnce();
//                    ZTarVision = getAngle() - visionAngle;
//                }
//                pressed_Once_Vision = false;
//                visionTelemetry = true;
//                sleep(200);
//            } if(gamepad1.x && pressed_Once_Vision2 == true) {
//                pressed_Once_Vision2 = false;
//                visionTelemetry = false;
//                useVisionShoot = false;
//                this.isDetected = false;
//                sendMotorCommands = true;
//                visionSuccessful = false;
//                pressed_Once_Vision = true;
//                //  kickerServo.setPosition(-1);
//            }





            //add telemetry


//            telemetry.addData("IsPressed", pressedOnce);
//            telemetry.addData("X", FX);
//            telemetry.addData("Y", FY);
//            telemetry.addData("CA", CA);
//            telemetry.addData("RSA", RSA);
//            telemetry.addData("RA", getAngle());
//
//            telemetry.addData("zMag", zMag);
//            telemetry.addData("ZTar", ZTar);
//
//            telemetry.addData("FREV", frontRightMotor.getCurrentPosition());
//            telemetry.addData("FLEV", frontLeftMotor.getCurrentPosition());
//            telemetry.addData("RREV", rearRightMotor.getCurrentPosition());
//            telemetry.addData("RLEV", rearLeftMotor.getCurrentPosition());

//            if(!visionTelemetry) {
//                telemetry.addData("Status", "Manual Shooting");
//                telemetry.addData("Average I", (NerdVelocityFollowing_Teleop.FLTotalError+NerdVelocityFollowing_Teleop.FRTotalError+NerdVelocityFollowing_Teleop.RLTotalError+NerdVelocityFollowing_Teleop.RRTotalError)/4);
//                telemetry.update();
//            }




        //    telemetry.addData("Status", "Running");

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
        double DBanMin = -.1;
        double DBanMax = .1;
        int MaxError = 10;
        double error = 0;
        double speed = 0;
        double TotalError = 0;
        double PrevError = 0;
        double MaxSpeed = 0;


        if(useVisionShoot && visionSuccessful) {
            TotalError = ZTotalError_Vision;
        }
        else {
            TotalError = ZTotalError;
        }
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


//        if(useVisionShoot)
//            ZSpeed = -speed;
//        else


            ZSpeed = speed;

        if(useVisionShoot && visionSuccessful) {
            ZTotalError_Vision = TotalError;
        }
        else {
            ZTotalError = TotalError;
        }
        ZPrevError = PrevError;


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



    private double getAngleOnce() {

        double angleToTarget = 0;
        useVisionShoot = true;
        recognition = nerdtfObjectDetector.detect("BlueGoal", true);

        if (recognition != null) {

            telemetry.addData("Angle to Object, plus, it freaking works", recognition.estimateAngleToObject(AngleUnit.DEGREES));
            telemetry.addData("Image Center", nerdtfObjectDetector.findImageCenter(recognition));
            telemetry.addData("Object Center", nerdtfObjectDetector.findObjectCenter(recognition));
            telemetry.addData("Distance", nerdtfObjectDetector.findDistanceToObjectOne(recognition));
            telemetry.addData("Width in Pixels", recognition.getWidth());
            telemetry.update();


//            boolean isTargetAligned=false;

            recognition = nerdtfObjectDetector.detect("BlueGoal", true);

            if (recognition != null) {
                //count++;


           //     angleToTarget = recognition.estimateAngleToObject(AngleUnit.DEGREES) - angleOffsetVision;

                angleToTarget = doStandardDeviation(10, 3);

                if(Math.abs(angleToTarget) <= 10) {
                    angleOffsetVision = -2;
                } else if(Math.abs(angleToTarget) > 10 && Math.abs(angleToTarget) <= 15) {
                    angleOffsetVision = -3;
                } else if(Math.abs(angleToTarget) > 15 && Math.abs(angleToTarget) <= 20){
                    angleOffsetVision = -4;
                } else {
                    angleOffsetVision = - 5;
                }



                telemetry.addData("Angle to target", angleToTarget);
                telemetry.update();




//                nerdShooterClass.indexRings();
//                isTargetAligned = true;

                visionSuccessful = true;
            }
            else{
                telemetry.addData("Second Detection Failed", "oof");
                telemetry.update();
                visionSuccessful=false;
            }
        }
        else {
            telemetry.addData("it didnt find it", "sucks for u");
            visionSuccessful = false;

        }

        return angleToTarget-angleOffsetVision;


    }

    public double doStandardDeviation(int numberOfReadings, int toleranceInDegrees) {
        double[] readings = new double[numberOfReadings];
        double average;
        double sum = 0;
        for(int i = 0; i <= numberOfReadings - 1; i++) {
            recognition = nerdtfObjectDetector.detect("BlueGoal", true);
            readings[i] = recognition.estimateAngleToObject(AngleUnit.DEGREES);
        }
        for(int i2 = 0; i2 <= numberOfReadings - 1; i2++) {
            sum = sum + readings[i2];
        }
        average = sum / numberOfReadings;
        sum = 0;
        for(int i3 = 0; i3 <= numberOfReadings - 1; i3++) {
            if(Math.abs(average - readings[i3]) > toleranceInDegrees) {
                readings[i3] = 0;
            }
        }
        for(int i4 = 0; i4 <= numberOfReadings - 1; i4++) {
            sum = sum + readings[i4];
        }
        return sum / numberOfReadings;
    }





}

