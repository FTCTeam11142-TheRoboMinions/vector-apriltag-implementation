package org.firstinspires.ftc.teamcode;
import java.lang.Math.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="Q RED Warehouse")
public class Q_RED_Warehouse extends LinearOpMode {
    DcMotor fY, fX, bY, bX, linSlide, inCar;
    Servo box;
    double fYPower = 0;
    double bYPower = 0;

    //Gyroscope Initialization
    private BNO055IMU imu;
    double absHeading;
    Orientation angles;
    double targetHeading = 0;
    int errorScaler = 35;


    //SampleMecanumDrive enigma;
    /* Note: This sample uses the all-objects Tensor Flow model (FreightFrenzy_BCDM.tflite), which contains
     * the following 4 detectable objects
     *  0: Ball,
     *  1: Cube,
     *  2: Duck,
     *  3: Marker (duck location tape marker)
     *
     *  Two additional model assets are available which only contain a subset of the objects:
     *  FreightFrenzy_BC.tflite  0: Ball,  1: Cube
     *  FreightFrenzy_DM.tflite  0: Duck,  1: Marker
     */
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };
    // Import the variables from the util class for the motors and servos

    // Declare counters at natural starting point
    /*
    As the Op mode runs, either of the integer counters will increase based on whether an object is detected or absent.
    The variable with a higher count after 10 loops determines the likelihood of an object actually present in the webcam's view.
     */
    int objectDetected = 0;
    int objectAbsent= 0;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AZngU9H/////AAABmWcUJSDyAkXTsV1ZbtiI0c6Ff6REKfqt0K5ILdhwDfDX/lDPPESZZnlQJOWzJd+q4CIJ4ExzUj5i92YJQrebNEBqaAR03Xf8OWDWgj8MDNrfa8Wu3FGtsp6fyrFUy+f7lNhfC/4TSmk1zVkFxyK34H+mW3LeeChBThKM6bvC3d7SV6bxSwcSXAfN9ZFMbxpMVCjW/J8TO/MmABRBqU27CvwM59zhvQknC3euq85hQS44i86KVggsajKQ0NEEdRjfF0WVchimOvOGXHjCVmQ2sPiXGITzcpuX92ifC8t/gJLcbQCvB6goatUrWY6AB9VYJIQpSXKD97/nmAO/nCVIHJ/mIZ6Ed82hR4kwNXIULe+A";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();
        //robot inputs all defined variables from the SampleMecanumrobot class

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it he
         *
         * re so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(2, 16/9);

            // Only a single object should be visible in the webcam's view, so the viewing area is squared.
        }
        fY = hardwareMap.dcMotor.get("rf");
        bY = hardwareMap.dcMotor.get("lr");
        fX = hardwareMap.dcMotor.get("lf");
        bX = hardwareMap.dcMotor.get("rr");
        linSlide = hardwareMap.get(DcMotor.class, "linx");
        inCar = hardwareMap.get(DcMotor.class, "carin");
        box = hardwareMap.get(Servo.class, "hopper");
        fX.setDirection(DcMotorSimple.Direction.REVERSE);
        bY.setDirection(DcMotorSimple.Direction.REVERSE);
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

        /** Wait for the game to begin */
        telemetry.addData(">", "Autonomous Ready");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            for(int ti = 1; ti < 20; ti++) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                            i++;

                            // check label to see if the camera now sees a Duck
                            if (recognition.getLabel().equals("Cube")) {
                                telemetry.addData("Object Detected", "TEAM MARKER");
                                objectDetected++;
                            } else {
                                objectAbsent++;
                            }
                        }
                        telemetry.update();
                    }
                }
            }

            if (objectDetected > objectAbsent) {
                // The object is located on the right-most spot
                telemetry.addData("Position: ", "Right");
                telemetry.update();
                tfod.shutdown();
                sleep(500);
                //Call object right method here
                objectRight();
            }
            else  {
                // The object is not on the left-most spot, so we move left to the middle spot
                //add left movement here
                Drive(0.3,0,9);
                sleep(750);
                for(int ti = 1; ti < 20; ti++) {
                    if (tfod != null) {
                        // getUpdatedRecognitions() will return null if no new information is available since
                        // the last time that call was made.
                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                        if (updatedRecognitions != null) {
                            telemetry.addData("# Object Detected", updatedRecognitions.size());

                            // step through the list of recognitions and display boundary info.
                            int i = 0;
                            for (Recognition recognition : updatedRecognitions) {
                                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                        recognition.getLeft(), recognition.getTop());
                                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                        recognition.getRight(), recognition.getBottom());
                                i++;

                                // check label to see if the camera now sees a Duck
                                if (recognition.getLabel().equals("Cube")) {
                                    telemetry.addData("Object Detected", "TEAM MARKER");
                                    objectDetected++;
                                } else {
                                    objectAbsent++;
                                }
                            }
                            telemetry.update();
                        }
                    }
                }

                if (objectDetected > objectAbsent) {
                    // The object is detected on the middle spot
                    telemetry.addData("Position: ", "Middle");
                    telemetry.update();
                    tfod.shutdown();
                    sleep(500);

                    //Call object middle
                    objectMiddle();
                }
                else {
                    // The object was not detected in the middle, so it must be on the left spot
                    telemetry.addData("Position: ", "Left");
                    telemetry.update();
                    tfod.shutdown();
                    sleep(500);
                    //call object right
                    objectLeft();

                }
            }
        }
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    // Power, X, Y

    //Freight Left
    public void objectRight(){
        Drive(0.25, 0, -14 );         //to carousel
        Drive(0.30, 24, 0);          //
        Correct(0.0);
        inCar.setPower(1);
        sleep(1500);
        inCar.setPower(0);
        Correct(0);
        Drive(0.35, -7 , 0 );
        Correct(0);
        Drive(0.35, -7 , 0 );
        Correct(0);
        Drive(0.35, -7 , 0 );
        Correct(0);
        Drive(0.35, -9 , 0 );
        Correct(0);
        Drive(0.35, 0, 85);
    }

    public void objectMiddle(){
        Drive(0.22, 0, -21 );         //to carousel
        Correct(0);
        Drive(0.35, 21, 0);          //to hub
        Correct(0.0);
        inCar.setPower(1);
        Extend(0.6, 6);
        sleep(1500);
        inCar.setPower(0);
        Correct(0);
        Drive(0.35, -7 , 0 );
        Correct(0);
        Drive(0.35, -7 , 0 );
        Correct(0);
        Drive(0.35, -7 , 0 );
        Correct(0);
        Drive(0.35, -9 , 0 );
        Correct(0);
        Drive(0.35, 0, 85);
    }

    //RIGHT Freight
    public void objectLeft(){
        Drive(0.25, 0, -21 );         //inline with hub
        Drive(0.35, 23, 0);          //
        Correct(0.0);
        inCar.setPower(1);
        Extend(0.8, 13);
        telemetry.addData("Sleepy","Time" );
        telemetry.update();
        sleep(1500);
        inCar.setPower(0);
        Extend(0.8, -13);
        inCar.setPower(0);
        Correct(0);
        Drive(0.35, -7 , 0 );
        Correct(0);
        Drive(0.35, -7 , 0 );
        Correct(0);
        Drive(0.35, -7 , 0 );
        Correct(0);
        Drive(0.35, -9 , 0 );
        Correct(0);
        Drive(0.35, 0, 85);
    }

    public void Drive (double inPower, double Xdistance, double Ydistance) {
        //reset
        fY.setMode(RunMode.STOP_AND_RESET_ENCODER);
        bY.setMode(RunMode.STOP_AND_RESET_ENCODER);
        fX.setMode(RunMode.STOP_AND_RESET_ENCODER);
        bX.setMode(RunMode.STOP_AND_RESET_ENCODER);

        //target position
        fY.setTargetPosition((int)Ydistance*50);
        bY.setTargetPosition((int)Ydistance*50);
        fX.setTargetPosition(-(int)Xdistance*50);
        bX.setTargetPosition(-(int)Xdistance*50);

        fY.setMode(RunMode.RUN_TO_POSITION);
        bY.setMode(RunMode.RUN_TO_POSITION);
        fX.setMode(RunMode.RUN_TO_POSITION);
        bX.setMode(RunMode.RUN_TO_POSITION);

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
            fY.setPower(inPower + ((absHeading-targetHeading)/errorScaler));
            bY.setPower(inPower - ((absHeading-targetHeading)/errorScaler));
            fX.setPower(inPower - ((absHeading-targetHeading)/errorScaler));
            bX.setPower(inPower + ((absHeading-targetHeading)/errorScaler));
            /*
            //Out of Bounds Correction
            if((absHeading-targetHeading)>5 || (absHeading-targetHeading)<-5){
                StopDriving(); //Stop robot to correct to prevent major course alterations
                while((absHeading-targetHeading)>2 || (absHeading-targetHeading)<-2){
                    if(fX.getTargetPosition() <= fX.getCurrentPosition() && bX.getTargetPosition() <= bX.getCurrentPosition()){
                        fX.setMode(RunMode.RUN_WITHOUT_ENCODER);
                        bX.setMode(RunMode.RUN_WITHOUT_ENCODER);
                    }
                    if((absHeading-targetHeading) > 0){
                        fX.setPower(0.15 + ((absHeading-targetHeading)/errorScaler));
                        bX.setPower(-0.15 - ((absHeading-targetHeading)/errorScaler));
                    }
                    else if((absHeading-targetHeading < 0)){
                        fX.setPower(-0.15 + ((absHeading-targetHeading)/errorScaler));
                        bX.setPower(0.15 - ((absHeading-targetHeading)/errorScaler));
                    }
                    telemetry.addData("Front X Power:", fX.getPower());
                    telemetry.addData("Back X Power:", bX.getPower());
                    telemetry.update();
                }

            }*/
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

    public void Extend (double extendPower, int extendDistance){
        //reset encoder
        linSlide.setMode(RunMode.STOP_AND_RESET_ENCODER);


        //set target position
        linSlide.setTargetPosition(extendDistance*75);


        linSlide.setMode(RunMode.RUN_TO_POSITION);

        //set power
        linSlide.setPower(extendPower);

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        while ((runtime.seconds() < 2.0) || linSlide.isBusy()){
            telemetry.addData("Extension ",linSlide.getCurrentPosition());
            telemetry.addData("TIMER", runtime);
            inCar.setPower(1);
            if (!linSlide.isBusy()){
                linSlide.setPower(0);
            }
            telemetry.update();
        }
        linSlide.setPower(0);
    }

    public void StopDriving(){
        fY.setPower(0);
        fX.setPower(0);
        bY.setPower(0);
        bX.setPower(0);

        linSlide.setPower(0);
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
            fX.setMode(RunMode.RUN_WITHOUT_ENCODER);
            bX.setMode(RunMode.RUN_WITHOUT_ENCODER);
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
    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "frontCam");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }


}