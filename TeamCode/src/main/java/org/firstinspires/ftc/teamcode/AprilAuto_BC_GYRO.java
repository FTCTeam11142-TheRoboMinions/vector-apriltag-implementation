/*
 * Copyright (c) 2021 OpenFTC Team
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

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class AprilAuto_BC_GYRO extends LinearOpMode
{
    DcMotor fY, fX, bY, bX, linSlide, inCar;
    Servo box, cap;
    double fYPower = 0;
    double bYPower = 0;

    //Gyroscope Initialization
    private BNO055IMU imu;
    double absHeading;
    Orientation angles;
    double targetHeading = 0;
    int errorScaler = 105;
    int YScaler = 15;
    int XScaler = 10;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.0762;

    int ID_TAG_OF_INTEREST = 11; // Tag ID 11 from the 36h11 family

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        fY = hardwareMap.dcMotor.get("rf");
        bY = hardwareMap.dcMotor.get("lr");
        fX = hardwareMap.dcMotor.get("lf");
        bX = hardwareMap.dcMotor.get("rr");
        linSlide = hardwareMap.get(DcMotor.class, "linx");
        inCar = hardwareMap.get(DcMotor.class, "carin");
        box = hardwareMap.get(Servo.class, "hopper");
        cap = hardwareMap.get(Servo.class, "capper");
        fY.setDirection(DcMotorSimple.Direction.REVERSE);
        bX.setDirection(DcMotorSimple.Direction.REVERSE);
        fX.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bX.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fX.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bX.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Gyroscope parameters
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        box.setPosition(0.25);
        cap.setPosition(0.91);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == ID_TAG_OF_INTEREST)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);

                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");

                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {

                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if(tagOfInterest == null)
        {
            /*
             * Insert your autonomous code here, presumably running some default configuration
             * since the tag was never sighted during INIT
             */
            caseRight();
        }
        else
        {
            /*
             * Insert your autonomous code here, probably using the tag pose to decide your configuration.
             */


            // left of camera is negative, right is positive pose
            if(tagOfInterest.pose.x*FEET_PER_METER <= 0)
            {
                // do something
                caseLeft();
            }
            else
            {
                // do something else
                caseMiddle();
            }
        }


        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        //while (opModeIsActive()) {sleep(20);}
    }

    // Display active positions of Apriltag pose
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    // Sample Roadrunner Movement >>>

    //  Trajectory strafePosOne = vector.trajectoryBuilder(new Pose2d(0, 0, 0))
    //          .lineToLinearHeading(new Pose2d(15, 15, Math.toRadians(0)))
    //          .build();
    //  vector.followTrajectory(strafePosOne);

    //For Roadrunner Trajectories:
    //Forward is +x +y
    //Backward is -x -y
    //Left is +x -y
    //Right is -x +y

    //Each method is used for the marker staring position
    public void caseLeft() {

    }

    public void caseMiddle() {

    }

    public void caseRight() {

        Drive(0.4,0, 8);
        Correct(0);
        Drive(0.4,-37, 0);
        Correct(0);
        Drive(0.4,0, 6);
        vectorTurn(0);
        highDeposit();
        Drive(0.4,65,0);
        vectorTurn(0);
        inCar.setPower(0.4);
        Drive(0.3,0,-15);
        vectorTurn(0);
        sleep(3500);
        inCar.setPower(0);
        sleep(500);
        Drive(0.6,0,11.3);
    }

    public void Drive (double inPower, double Xdistance, double Ydistance) {
        //reset
        fY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //target position
        fY.setTargetPosition((int)Ydistance*50);
        bY.setTargetPosition((int)Ydistance*50);
        fX.setTargetPosition(-(int)Xdistance*50);
        bX.setTargetPosition(-(int)Xdistance*50);

        fY.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bY.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fX.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bX.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set power
        if(Xdistance == 0){
            fY.setPower(inPower);
            bY.setPower(inPower);
        }
        else if(Ydistance == 0){
            fX.setPower(inPower);
            bX.setPower(inPower);
        }
        else{
            fY.setPower(inPower);
            bY.setPower(inPower);
            fX.setPower(inPower);
            bX.setPower(inPower);
        }

        //Active Gyroscopic corrections while running to position
        while (fX.isBusy() || bX.isBusy() || fY.isBusy() || bY.isBusy()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Heading", angles.firstAngle);
            telemetry.addData("Target", targetHeading);
            absHeading = angles.firstAngle;
            //Standard Correction of each wheel by adding weighted power
            fY.setPower(inPower + ((absHeading-targetHeading)/YScaler));
            bY.setPower(inPower - ((absHeading-targetHeading)/YScaler));
            fX.setPower(inPower + ((absHeading-targetHeading)/XScaler));
            bX.setPower(inPower - ((absHeading-targetHeading)/XScaler));
            telemetry.addData("f X current:", fX.getCurrentPosition());
            telemetry.addData("b X current:", bX.getCurrentPosition());
            telemetry.addData("f Y current:", fY.getCurrentPosition());
            telemetry.addData("b Y current:", bY.getCurrentPosition());
            telemetry.addData("f X target:", fX.getTargetPosition());
            telemetry.addData("b X target:", bX.getTargetPosition());
            telemetry.addData("f Y target:", fY.getTargetPosition());
            telemetry.addData("b Y target:", bY.getTargetPosition());
            telemetry.addData("f Y Power:", fY.getPower());
            telemetry.addData("b Y Power:", bY.getPower());
            telemetry.addData("f X Power:", fX.getPower());
            telemetry.addData("b X Power:", bX.getPower());
            telemetry.update();
        }
        StopDriving();
    }

    public void vectorTurn (double correctTo){
        targetHeading = (correctTo);
        while((absHeading-targetHeading)>2.5 || (absHeading-targetHeading)<-2.5){
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Heading", angles.firstAngle);
            telemetry.addData("Target", targetHeading);
            absHeading = angles.firstAngle;
            fX.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            bX.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            double bXPow = 0;
            double fXPow = 0;
            if((absHeading-targetHeading) > 0){
                bXPow = (0.2 +((absHeading-targetHeading)/errorScaler));
                if (bXPow > 0.6){
                    bXPow = 0.6;
                }
                else if (bXPow < -0.6){
                    bXPow = -0.6;
                }

                fXPow = (-0.2 -((absHeading-targetHeading)/errorScaler));
                if (fXPow > 0.6){
                    fXPow = 0.6;
                }
                else if (fXPow < -0.6){
                    fXPow = -0.6;
                }
                fX.setPower(fXPow);
                bX.setPower(bXPow);
            }
            else if((absHeading-targetHeading) < 0){
                bXPow = (-0.2 +((absHeading-targetHeading)/errorScaler));
                if (bXPow > 0.6){
                    bXPow = 0.6;
                }
                else if (bXPow < -0.6){
                    bXPow = -0.6;
                }

                fXPow = (0.2 -((absHeading-targetHeading)/errorScaler));
                if (fXPow > 0.6){
                    fXPow = 0.6;
                }
                else if (fXPow < -0.6){
                    fXPow = -0.6;
                }

                fX.setPower(fXPow);
                bX.setPower(bXPow);
            }
            telemetry.addData("Front X Power:", fX.getPower());
            telemetry.addData("Back X Power:", bX.getPower());
            telemetry.update();
        }
        StopDriving();
    }

    public void StopDriving(){
        fY.setPower(0);
        fX.setPower(0);
        bY.setPower(0);
        bX.setPower(0);
    }

    public void Correct(double correctTo){
        targetHeading = (correctTo);
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while((runtime.seconds() < 2.0) &&((absHeading-targetHeading)>2 || (absHeading-targetHeading)<-2)){
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Heading", angles.firstAngle);
            telemetry.addData("Target", targetHeading);
            absHeading = angles.firstAngle;
            fX.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            bX.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if((absHeading-targetHeading) > 0){
                double fXPow = (0.25 +((absHeading-targetHeading)/errorScaler));
                if (fXPow > 0.6){
                    fXPow = 0.6;
                }
                else if (fXPow < -0.6){
                    fXPow = -0.6;
                }
                double bXPow = (-0.25 -((absHeading-targetHeading)/errorScaler));
                if (bXPow > 0.6){
                    bXPow = 0.6;
                }
                else if (bXPow < -0.6){
                    bXPow = -0.6;
                }
                fX.setPower(fXPow);
                bX.setPower(bXPow);
            }
            else if((absHeading-targetHeading < 0)){
                double fXPow = (-0.25 +((absHeading-targetHeading)/errorScaler));
                if (fXPow > 0.6){
                    fXPow = 0.6;
                }
                else if (fXPow < -0.6){
                    fXPow = -0.6;
                }
                double bXPow = (0.25 -((absHeading-targetHeading)/errorScaler));
                if (bXPow > 0.6){
                    bXPow = 0.6;
                }
                else if (bXPow < -0.6){
                    bXPow = -0.6;
                }
                fX.setPower(fXPow);
                bX.setPower(bXPow);
            }
            telemetry.addData("Front X Power:", fX.getPower());
            telemetry.addData("Back X Power:", bX.getPower());
            telemetry.update();
        }
        StopDriving();
    }

    public void linearExtension (double velocity, int position) {
        //reset encoder
        linSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //set target position between limits 0 to 1500
        //set negative position with positive power to reverse slide direction
        linSlide.setTargetPosition(position);
        linSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //set power
        linSlide.setPower(velocity);

        while (linSlide.isBusy()){

        }
        linSlide.setPower(0);
    }

    public void carouselCycle () {
        //accelerate carousel wheel
        inCar.setPower(0.5);
        sleep(1000);
        //slow down carousel wheel so duck is not launched
        inCar.setPower(0.25);
        sleep(1500);
        linSlide.setPower(0);
    }

    public void highDeposit() {
        linearExtension(0.8, -1500);
        box.setPosition(0.825);
        sleep(300);
        linearExtension(0.8, 100);
        box.setPosition(0);
        linearExtension(0.8, 1450);
        //box.setPosition(0.05);
    }

    public void middleDeposit() {
        linearExtension(0.5, -750);
        box.setPosition(0.75);
        sleep(500);
        //Trajectory forwardDrop = vector.trajectoryBuilder(new Pose2d(0, 0, 0))
        //        .lineToLinearHeading(new Pose2d(5, 5, Math.toRadians(0)))
        //        .build();
        //vector.followTrajectory(forwardDrop);
        box.setPosition(0.825);
        sleep(500);
    }

    public void lowerDeposit() {
        linearExtension(0.5, -500);
        box.setPosition(0.75);
        sleep(500);
        //Trajectory forwardDrop = vector.trajectoryBuilder(new Pose2d(0, 0, 0))
        //        .lineToLinearHeading(new Pose2d(5, 5, Math.toRadians(0)))
        //        .build();
        //vector.followTrajectory(forwardDrop);
        box.setPosition(0.825);
        sleep(500);
    }

    public void middleRetraction() {
        box.setPosition(0.25);
        sleep(500);
        linearExtension(0.5, 750);
        sleep(500);
    }

    public void lowerRetraction() {
        box.setPosition(0.25);
        sleep(500);
        linearExtension(0.5, 500);
        sleep(500);
    }

}