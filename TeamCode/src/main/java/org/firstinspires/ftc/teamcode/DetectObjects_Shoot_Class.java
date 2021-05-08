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

import android.media.MediaSync;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

//@TeleOp
public class DetectObjects_Shoot_Class
{



    private HardwareMap hardwareMap;
    LinearOpMode opMode;

    OpenCvCamera phoneCam;
    SkystoneDeterminationPipeline pipeline;

    private ElapsedTime NERDTimer = new ElapsedTime();

    public DetectObjects_Shoot_Class(LinearOpMode opmode) {
        this.opMode = opmode;
        this.hardwareMap = opmode.hardwareMap;
    }

    public void initialize() {
        //Change the deviceName parameter to whatever the motor/servo is named in your config
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

      //  nerdShooterClass.initialize();
    }

    //@Override
//    public void runShoot() {
//        nerdShooterClass.runShooterMotor(0.92);
//        this.opMode.sleep(3000);
//        nerdShooterClass.indexRings();
//    }

    //@Override
    public int runDetect()
    {
        int ringsNum=0;

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

        this.NERDTimer.reset();
        while (this.NERDTimer.seconds() <= 5 && !opMode.isStopRequested())
        {
            this.opMode.telemetry.addData("Ring Analysis", pipeline.avg_ring);
            this.opMode.telemetry.addData("Number of Rings : ", pipeline.position_ring);
            this.opMode.telemetry.addData("Wobble : ", pipeline.wobble_position);
            this.opMode.telemetry.addData("Powershot : ",pipeline.position_power);
            this.opMode.telemetry.update();

            //if(pipeline.position_power == SkystoneDeterminationPipeline.PowerPosition.BLUE) {
                this.opMode.telemetry.addData("BLUE Power Shot Detected : ",pipeline.position_power);
            //}

            // Don't burn CPU cycles busy-looping in this sample
//            this.opMode.sleep(50);
            if(pipeline.position_ring == SkystoneDeterminationPipeline.RingPosition.FOUR) {
                ringsNum = 4;
            }
            else if(pipeline.position_ring == SkystoneDeterminationPipeline.RingPosition.ONE) {
                ringsNum = 1;
            }
            else {
                ringsNum = 0;
            }
        }

//      phoneCam.closeCameraDeviceAsync(closeListener);
        phoneCam.stopRecordingPipeline();
        phoneCam.stopStreaming();
        phoneCam.closeCameraDevice();


        return ringsNum;
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
        final int ONE_RING_THRESHOLD = 120;
        //final int ONE_RING_THRESHOLD = 115;

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

        final int POWER_THRESHOLD1 = 175; //Red
        final int POWER_THRESHOLD2 = 135; //Blue

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
        private volatile RingPosition position_ring
                = RingPosition.NONE;

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
        private volatile WobblePosition wobble_position = WobblePosition.NONE;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        Mat region1_Cb_P;
        Mat YCrCb_P = new Mat();
        Mat Cb_P = new Mat();
        int avg_Power;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile PowerPosition position_power = PowerPosition.NONE;

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

            //position_ring = DetectObjects_Shoot_Class.SkystoneDeterminationPipeline.RingPosition.FOUR; // Record our analysis
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

            /*Imgproc.rectangle(
                    input, // Buffer to draw on
                    region_wobble_pointA, // First point which defines the rectangle
                    region_wobble_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines
*/
            wobble_position = WobblePosition.RED; // Record our analysis
            if(avg_wobble > RED_WOBBLE_THRESHOLD) {
                wobble_position = WobblePosition.RED;
            }else if(avg_BlueWobble > BLUE_WOBBLE_THRESHOLD){
                wobble_position = WobblePosition.BLUE;
            }else{
                wobble_position = WobblePosition.NONE;
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

            position_power = PowerPosition.BLUE; // Record our analysis
            if(avg_Power > POWER_THRESHOLD2){
                position_power = PowerPosition.BLUE;
            }else{
                position_power = PowerPosition.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA_P, // First point which defines the rectangle
                    region1_pointB_P, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill


            if(position_power == PowerPosition.BLUE){
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