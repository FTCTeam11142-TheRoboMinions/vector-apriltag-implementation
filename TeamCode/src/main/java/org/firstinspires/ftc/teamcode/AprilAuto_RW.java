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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

@Autonomous
public class AprilAuto_RW extends LinearOpMode
{
    SampleMecanumDrive vector;
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
        vector = new SampleMecanumDrive(hardwareMap);

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
        Trajectory diagonal = vector.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(26, -6, Math.toRadians(0)))
                .build();
        Trajectory forward = vector.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(5, 5, Math.toRadians(0)))
                .build();
        Trajectory side = vector.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(5, -5, Math.toRadians(0)))
                .build();
        Trajectory toHub = vector.trajectoryBuilder(new Pose2d(0, 0, 0))
                  .lineToLinearHeading(new Pose2d(10, 10, Math.toRadians(0)))
                  .build();
        Trajectory backToWall = vector.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(-12.5, -12.5, Math.toRadians(0)))
                .build();
        Trajectory strafeToHub = vector.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(15, -15, Math.toRadians(0)))
                .build();
        Trajectory strafeToWall = vector.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(25, -25, Math.toRadians(0)))
                .build();
        Trajectory forwardToWarehouse = vector.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(-35, -35, Math.toRadians(0)))
                .build();
        Trajectory outFromWall = vector.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(-6, 6, Math.toRadians(0)))
                .build();
        Trajectory postCollection = vector.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(22, 22, Math.toRadians(0)))
                .build();
        Trajectory strafeFromWall = vector.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(-20, 20, Math.toRadians(0)))
                .build();

        vector.capper.setPosition(0.7);
        vector.followTrajectory(diagonal);
        vectorTurn(0);
        highDeposit();
        vectorTurn(90);
        vector.followTrajectory(strafeToWall);
        vector.carin.setPower(1);
        vector.followTrajectory(forwardToWarehouse);
        sense();
        vector.hopper.setPosition(0.25);
        vectorTurn(90);
        vector.carin.setPower(-1);
        sleep(500);
        vector.carin.setPower(0);
        vector.followTrajectory(strafeToWall);
        vector.followTrajectory(postCollection);
        vector.followTrajectory(outFromWall);
        vectorTurn(45);
        vector.followTrajectory(toHub);
        highDeposit();
        vectorTurn(90);
        vector.followTrajectory(strafeToWall);
        vector.followTrajectory(forwardToWarehouse);
    }

    public void vectorDrive (double inPower, double Xdistance, double Ydistance) {
        //reset
        vector.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vector.leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vector.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vector.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //target position
        vector.rightFront.setTargetPosition((int)Ydistance*50);
        vector.leftRear.setTargetPosition((int)Ydistance*50);
        vector.leftFront.setTargetPosition(-(int)Xdistance*50);
        vector.rightRear.setTargetPosition(-(int)Xdistance*50);

        vector.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vector.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vector.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vector.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set power
        if(Xdistance == 0){
            vector.rightFront.setPower(inPower);
            vector.leftRear.setPower(inPower);
        }
        else if(Ydistance == 0){
            vector.leftFront.setPower(inPower);
            vector.rightRear.setPower(inPower);
        }
        else{
            vector.rightFront.setPower(inPower);
            vector.leftRear.setPower(inPower);
            vector.leftFront.setPower(inPower);
            vector.rightRear.setPower(inPower);
        }

        //Active Gyroscopic corrections while running to position
        while (vector.leftFront.isBusy() || vector.rightRear.isBusy() || vector.rightFront.isBusy() || vector.leftRear.isBusy()) {
            vector.angles = vector.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Heading", vector.angles.firstAngle);
            telemetry.addData("Target", vector.targetHeading);
            vector.absHeading = vector.angles.firstAngle;
            //Standard Correction of each wheel by adding weighted power
            vector.rightFront.setPower(inPower + ((vector.absHeading-vector.targetHeading)/vector.DriveScaler));
            vector.leftRear.setPower(inPower - ((vector.absHeading-vector.targetHeading)/vector.DriveScaler));
            vector.leftFront.setPower(inPower - ((vector.absHeading-vector.targetHeading)/vector.DriveScaler));
            vector.rightRear.setPower(inPower + ((vector.absHeading-vector.targetHeading)/vector.DriveScaler));
            telemetry.addData("f X current:", vector.leftFront.getCurrentPosition());
            telemetry.addData("b X current:", vector.rightRear.getCurrentPosition());
            telemetry.addData("f Y current:", vector.rightFront.getCurrentPosition());
            telemetry.addData("b Y current:", vector.leftRear.getCurrentPosition());
            telemetry.addData("f X target:", vector.leftFront.getTargetPosition());
            telemetry.addData("b X target:", vector.rightRear.getTargetPosition());
            telemetry.addData("f Y target:", vector.rightFront.getTargetPosition());
            telemetry.addData("b Y target:", vector.leftRear.getTargetPosition());
            telemetry.addData("f Y Power:", vector.rightFront.getPower());
            telemetry.addData("b Y Power:", vector.leftRear.getPower());
            telemetry.addData("f X Power:", vector.leftFront.getPower());
            telemetry.addData("b X Power:", vector.rightRear.getPower());
            telemetry.update();
        }
        //stop all motors after reaching position
        vector.rightFront.setPower(0);
        vector.leftFront.setPower(0);
        vector.leftRear.setPower(0);
        vector.rightRear.setPower(0);
    }

    public void vectorCorrect(double correctTo){
        vector.targetHeading = (correctTo);
        while((vector.absHeading-vector.targetHeading)>2.5 || (vector.absHeading-vector.targetHeading)<-2.5){
            vector.angles = vector.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Heading", vector.angles.firstAngle);
            telemetry.addData("Target", vector.targetHeading);
            vector.absHeading = vector.angles.firstAngle;
            vector.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            vector.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if((vector.absHeading-vector.targetHeading) > 0){
                double fXPow = (0.1 +((vector.absHeading-vector.targetHeading)/vector.errorScaler));
                if (fXPow > 0.5){
                    fXPow = 0.5;
                }
                else if (fXPow < -0.5){
                    fXPow = -0.5;
                }
                double bXPow = (-0.1 -((vector.absHeading-vector.targetHeading)/vector.errorScaler));
                if (bXPow > 0.5){
                    bXPow = 0.5;
                }
                else if (bXPow < -0.5){
                    bXPow = -0.5;
                }
                vector.leftFront.setPower(fXPow);
                vector.rightRear.setPower(bXPow);
            }
            else if((vector.absHeading-vector.targetHeading < 0)){
                double fXPow = (-0.1 +((vector.absHeading-vector.targetHeading)/vector.errorScaler));
                if (fXPow > 0.5){
                    fXPow = 0.5;
                }
                else if (fXPow < -0.5){
                    fXPow = -0.5;
                }
                double bXPow = (0.1 -((vector.absHeading-vector.targetHeading)/vector.errorScaler));
                if (bXPow > 0.5){
                    bXPow = 0.5;
                }
                else if (bXPow < -0.5){
                    bXPow = -0.5;
                }
                vector.leftFront.setPower(-fXPow);
                vector.rightRear.setPower(-bXPow);
            }
            telemetry.addData("Front X Power:", vector.leftFront.getPower());
            telemetry.addData("Back X Power:", vector.rightRear.getPower());
            telemetry.update();
        }
        //stop all motors after reaching position
        vector.rightFront.setPower(0);
        vector.leftFront.setPower(0);
        vector.leftRear.setPower(0);
        vector.rightRear.setPower(0);
    }

    public void vectorTurn (double correctTo){
            vector.targetHeading = (correctTo);
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while((runtime.seconds() < 1.5) && ((vector.absHeading-vector.targetHeading)>2.5 || (vector.absHeading-vector.targetHeading)<-2.5)){
                vector.angles = vector.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                telemetry.addData("Heading", vector.angles.firstAngle);
                telemetry.addData("Target", vector.targetHeading);
                vector.absHeading = vector.angles.firstAngle;
                vector.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                vector.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                double bXPow = 0;
                double fXPow = 0;
                if((vector.absHeading-vector.targetHeading) > 0){
                    bXPow = (0.2 +((vector.absHeading-vector.targetHeading)/vector.errorScaler));
                    if (bXPow > 0.7){
                        bXPow = 0.7;
                    }
                    else if (bXPow < -0.7){
                        bXPow = -0.7;
                    }

                    fXPow = (-0.2 -((vector.absHeading-vector.targetHeading)/vector.errorScaler));
                    if (fXPow > 0.7){
                        fXPow = 0.7;
                    }
                    else if (fXPow < -0.7){
                        fXPow = -0.7;
                    }
                    vector.leftFront.setPower(fXPow);
                    vector.rightRear.setPower(bXPow);
                }
                else if((vector.absHeading-vector.targetHeading) < 0){
                    bXPow = (-0.2 +((vector.absHeading-vector.targetHeading)/vector.errorScaler));
                    if (bXPow > 0.7){
                        bXPow = 0.7;
                    }
                    else if (bXPow < -0.7){
                        bXPow = -0.7;
                    }

                    fXPow = (0.2 -((vector.absHeading-vector.targetHeading)/vector.errorScaler));
                    if (fXPow > 0.7){
                        fXPow = 0.7;
                    }
                    else if (fXPow < -0.7){
                        fXPow = -0.7;
                    }

                    vector.leftFront.setPower(fXPow);
                    vector.rightRear.setPower(bXPow);
                }
                telemetry.addData("Front X Power:", vector.leftFront.getPower());
                telemetry.addData("Back X Power:", vector.rightRear.getPower());
                telemetry.update();
            }

        //stop all motors after reaching position
        vector.rightFront.setPower(0);
        vector.leftFront.setPower(0);
        vector.leftRear.setPower(0);
        vector.rightRear.setPower(0);
    }

    public void linearExtension (double velocity, int position) {
        //reset encoder
        vector.linx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //set target position between limits 0 to 1500
        //set negative position with positive power to reverse slide direction
        vector.linx.setTargetPosition(position);
        vector.linx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //set power
        vector.linx.setPower(velocity);

           while (vector.linx.isBusy()){

          }
          vector.linx.setPower(0);
    }

    public void carouselCycle () {
        //accelerate carousel wheel
        vector.carin.setPower(0.5);
        sleep(1000);
        //slow down carousel wheel so duck is not launched
        vector.carin.setPower(0.25);
        sleep(1500);
        vector.linx.setPower(0);
    }

    public void highDeposit() {
        linearExtension(0.8, -1500);
        vector.hopper.setPosition(0.825);
        sleep(500);
        linearExtension(0.8, 100);
        vector.hopper.setPosition(0.25);
        linearExtension(0.8, 1500);
        vector.hopper.setPosition(0);
        sleep(500);
    }

    public void sense(){
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while((runtime.seconds() < 2) && (vector.freight == false)) {
            vectorTurn(100);
            if (vector.colorDistance instanceof DistanceSensor) {
                telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) vector.colorDistance).getDistance(DistanceUnit.CM));
                vector.boxDist = ((DistanceSensor) vector.colorDistance).getDistance(DistanceUnit.CM);
                telemetry.update();
                if (vector.boxDist < 5.75) {
                    vector.freight = true;
                    vector.hopper.setPosition(0.25);
                }
            }
            vectorTurn(80);
            vector.boxDist = ((DistanceSensor) vector.colorDistance).getDistance(DistanceUnit.CM);
            telemetry.update();
        }
    }



}