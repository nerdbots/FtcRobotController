/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Disabled
@TeleOp
public class DetectObjects_Shoot extends LinearOpMode
{
    //OpenCvInternalCamera phoneCam;
    OpenCvCamera phoneCam;
    SkystoneDeterminationPipeline pipeline;
    private DistanceSensor sensorRange;

    public static final double indexerHomePos = 1.0;
    public static final double indexerPushedPos = 0.45;
    public DcMotor shooterMotor;
    public Servo indexingServo;
    private NERDShooterClass nerdShooterClass;

    public void indexRings() {
        int count = 0;
        boolean cycle = true;
        indexingServo.setPosition(indexerHomePos);
        while (count < 6) {
            double position = 0.0;

            if (cycle) {
                //Make Indexer go forward
                position = this.indexerPushedPos;
                cycle = false;
            } else {
                //Make Indexer go backward
                position = this.indexerHomePos;
                cycle = true;
            }

            // Set the servo to the new position and pause;
            indexingServo.setPosition(position);
            sleep(500);
            count++;
        }
    }


        @Override
    public void runOpMode()
    {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
        //phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        shooterMotor = hardwareMap.get(DcMotor.class, "Front");
        indexingServo = hardwareMap.get(Servo.class, "indexingServo");
        indexingServo.setPosition(indexerHomePos);

        pipeline = new SkystoneDeterminationPipeline();
        phoneCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        //phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }
        });

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("Ring Analysis", pipeline.getAnalysis_ring());
            telemetry.addData("Number of Rings : ", pipeline.position_ring);
            //telemetry.addData("Wobble Analysis", pipeline.getAnalysis_wobble());
            telemetry.addData("Wobble : ", pipeline.wobble_position);
            //telemetry.addData("Powershot Analysis",pipeline.getAnalysis_power());
            telemetry.addData("Powershot : ",pipeline.position_power);
            //telemetry.addData("Distance of Object detected (inch) : ", String.format("%.01f in", sensorRange.getDistance(DistanceUnit.INCH)));


            telemetry.update();
            //Shoot
            if(pipeline.position_power == DetectObjects_Shoot.SkystoneDeterminationPipeline.PowerPosition.BLUE) {
                telemetry.addData("Blue Powershot Detected : ",pipeline.position_power);
                shooterMotor.setPower(-4.75);
                sleep(10000);
                indexRings();
                shooterMotor.setPower(0);
            }

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }



    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the skystone position
         */
        public enum PowerPosition
        {
            BLUE,
            NONE
        }

        public enum RingPosition
        {
            FOUR,
            THREE,
            TWO,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);
        static final Scalar CLEAR = new Scalar(0, 0, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION_RING_TOPLEFT_ANCHOR_POINT = new Point(207,160);

        static final int REGION_WIDTH_RING = 35;
        static final int REGION_HEIGHT_RING = 35;

        final int FOUR_RING_THRESHOLD = 130;
        final int THREE_RING_THRESHOLD = 140;
        final int TWO_RING_THRESHOLD = 130;
        final int ONE_RING_THRESHOLD = 115;

        Point region_ring_pointA = new Point(
                REGION_RING_TOPLEFT_ANCHOR_POINT.x,
                REGION_RING_TOPLEFT_ANCHOR_POINT.y);
        Point region_ring_pointB = new Point(
                REGION_RING_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH_RING,
                REGION_RING_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT_RING);

        static final Point REGION1_TOPLEFT_ANCHOR_POINT_P = new Point(156,3);
        static final Point REGION2_TOPLEFT_ANCHOR_POINT_P = new Point(213,35);
        static final int REGION_WIDTH_P = 80;
        static final int REGION_HEIGHT_P = 15;
        static final int REGION2_HEIGHT_P = 25;
        static final int REGION2_WIDTH_P = 50;

        final int POWER_THRESHOLD1 = 175;
        final int POWER_THRESHOLD2 = 135;

        Point region1_pointA_P = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT_P.x,
                REGION1_TOPLEFT_ANCHOR_POINT_P.y);
        Point region1_pointB_P = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT_P.x + REGION_WIDTH_P,
                REGION1_TOPLEFT_ANCHOR_POINT_P.y + REGION_HEIGHT_P);
        Point region2_pointA_P = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT_P.x,
                REGION2_TOPLEFT_ANCHOR_POINT_P.y);
        Point region2_pointB_P = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT_P.x + REGION2_WIDTH_P,
                REGION2_TOPLEFT_ANCHOR_POINT_P.y + REGION2_HEIGHT_P);

        /*
         * Working variables
         */
        Mat region_ring_Cb;
        Mat YCrCb_ring = new Mat();
        Mat Cb_ring = new Mat();
        int avg_ring;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile RingPosition position_ring = RingPosition.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb_ring, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb_ring, Cb_ring, 1);
        }

        /* Wobble */
        public enum WobblePosition
        {
            RED,
            BLUE,
            NONE
        }
        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION_WOBBLE_TOPLEFT_ANCHOR_POINT = new Point(131,101);

        static final int REGION_WIDTH_WOBBLE = 30;
        static final int REGION_HEIGHT_WOBBLE = 15;

        final int RED_WOBBLE_THRESHOLD = 150;
        final int BLUE_WOBBLE_THRESHOLD = 135;

        Point region_wobble_pointA = new Point(
                REGION_WOBBLE_TOPLEFT_ANCHOR_POINT.x,
                REGION_WOBBLE_TOPLEFT_ANCHOR_POINT.y);
        Point region_wobble_pointB = new Point(
                REGION_WOBBLE_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH_WOBBLE,
                REGION_WOBBLE_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT_WOBBLE);

        /*
         * Working variables
         */
        Mat region_wobble_Cb;
        Mat YCrCb_wobble = new Mat();
        Mat Cb_wobble = new Mat();
        int avg_wobble;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile DetectObjects_Shoot.SkystoneDeterminationPipeline.WobblePosition wobble_position = DetectObjects_Shoot.SkystoneDeterminationPipeline.WobblePosition.NONE;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        Mat region1_Cb_P;
        Mat YCrCb_P = new Mat();
        Mat Cb_P = new Mat();
        int avg_Power;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile DetectObjects_Shoot.SkystoneDeterminationPipeline.PowerPosition position_power = DetectObjects_Shoot.SkystoneDeterminationPipeline.PowerPosition.NONE;

        void inputToCb_Power(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb_P, Imgproc.COLOR_BGR2YCrCb);
            Core.extractChannel(YCrCb_P, Cb_P, 1);
        }

        void inputToCb_WOBBLE(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb_wobble, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb_wobble, Cb_wobble, 1);
        }
        Mat region_BlueWobble_Cb;
        Mat YCrCb_BlueWobble = new Mat();
        Mat Cb_BlueWobble = new Mat();
        int avg_BlueWobble;

        void inputToCb_BlueWobble(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb_BlueWobble, Imgproc.COLOR_BGR2YCrCb);
            Core.extractChannel(YCrCb_BlueWobble, Cb_BlueWobble, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region_ring_Cb = Cb_ring.submat(new Rect(region_ring_pointA, region_ring_pointB));

            inputToCb_WOBBLE(firstFrame);

            region_wobble_Cb = Cb_wobble.submat(new Rect(region_wobble_pointA, region_wobble_pointB));

            inputToCb_BlueWobble(firstFrame);

            region_BlueWobble_Cb = Cb_BlueWobble.submat(new Rect(region_wobble_pointA, region_wobble_pointB));

            inputToCb_Power(firstFrame);

            region1_Cb_P = Cb_P.submat(new Rect(region1_pointA_P, region1_pointB_P));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg_ring = (int) Core.mean(region_ring_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region_ring_pointA, // First point which defines the rectangle
                    region_ring_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position_ring = RingPosition.FOUR; // Record our analysis
            if(avg_ring >= FOUR_RING_THRESHOLD) {
                position_ring = RingPosition.FOUR;
            }
            /*
            else if ((avg_ring > THREE_RING_THRESHOLD) && (avg_ring <= FOUR_RING_THRESHOLD)){
                position_ring = RingPosition.THREE;
            }
            else if ((avg_ring > TWO_RING_THRESHOLD) && (avg_ring <= THREE_RING_THRESHOLD)){
                    position_ring = RingPosition.TWO;
            }
            */
            else if ((avg_ring > ONE_RING_THRESHOLD)){
                position_ring = RingPosition.ONE;
            }else{
                position_ring = RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region_ring_pointA, // First point which defines the rectangle
                    region_ring_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

           // return input;



            inputToCb_WOBBLE(input);
            avg_wobble = (int) Core.mean(region_wobble_Cb).val[0];

            inputToCb_BlueWobble(input);
            avg_BlueWobble = (int) Core.mean(region_BlueWobble_Cb).val[0];

            /*
            Imgproc.rectangle(

                    input, // Buffer to draw on
                    region_wobble_pointA, // First point which defines the rectangle
                    region_wobble_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines
*/
            wobble_position = DetectObjects_Shoot.SkystoneDeterminationPipeline.WobblePosition.RED; // Record our analysis
            if(avg_wobble > RED_WOBBLE_THRESHOLD) {
                wobble_position = DetectObjects_Shoot.SkystoneDeterminationPipeline.WobblePosition.RED;
            }else if(avg_BlueWobble > BLUE_WOBBLE_THRESHOLD){
                wobble_position = DetectObjects_Shoot.SkystoneDeterminationPipeline.WobblePosition.BLUE;
            }else{
                wobble_position = DetectObjects_Shoot.SkystoneDeterminationPipeline.WobblePosition.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region_wobble_pointA, // First point which defines the rectangle
                    region_wobble_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            inputToCb_Power(input);

            avg_Power = (int) Core.mean(region1_Cb_P).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA_P, // First point which defines the rectangle
                    region1_pointB_P, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position_power = DetectObjects_Shoot.SkystoneDeterminationPipeline.PowerPosition.BLUE; // Record our analysis
            if(avg_Power > POWER_THRESHOLD2){
                position_power = DetectObjects_Shoot.SkystoneDeterminationPipeline.PowerPosition.BLUE;
            }else{
                position_power = DetectObjects_Shoot.SkystoneDeterminationPipeline.PowerPosition.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA_P, // First point which defines the rectangle
                    region1_pointB_P, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill


            if(position_power == DetectObjects_Shoot.SkystoneDeterminationPipeline.PowerPosition.BLUE){
                Imgproc.rectangle(
                        input,
                        region2_pointA_P,
                        region2_pointB_P,
                        GREEN,
                        2);

            }

            return input;
        }

        /* Wobble */

        public int getAnalysis_ring()
        {
            return avg_ring;
        }
        public int getAnalysis_wobble()
        {
            return avg_wobble;
        }
        public int getAnalysis_power()
        {
            return avg_Power;
        }
    }
}