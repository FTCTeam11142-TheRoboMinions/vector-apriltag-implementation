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

    @Override
    public void runOpMode() {
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
            Correct(90);

        }
    }
    // Power, X, Y

    //Freight Left
    public void objectLeft(){
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
    public void objectRight(){
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

    public void Turn (double correctTo){
        targetHeading = (correctTo);
        while((absHeading-targetHeading)>2.5 || (absHeading-targetHeading)<-2.5){
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
                double fXPow = (-0.2 +((absHeading-targetHeading)/errorScaler));
                if (fXPow > 0.6){
                    fXPow = 0.6;
                }
                else if (fXPow < -0.6){
                    fXPow = -0.6;
                }
                double bXPow = (0.2 -((absHeading-targetHeading)/errorScaler));
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
}