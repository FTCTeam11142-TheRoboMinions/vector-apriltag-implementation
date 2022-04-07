package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp(name = "327")
public class ThreeTwentySeven extends OpMode {
    public DcMotor leftFront, leftRear, rightRear, rightFront;
    public DcMotor linSlide, inCar;
    public Servo box;
    public ColorSensor colorDistance;
    double leftFrontPower, rightFrontPower, leftRearPower, rightRearPower, armPow, inCarPow, boxPos, boxDist;
    boolean freight = false;


    @Override
    public void init() {
        telemetry.clearAll();
        //Defining Hardware
        //Wheels
        leftFront = hardwareMap.get(DcMotorEx.class, "lf");
        leftRear = hardwareMap.get(DcMotorEx.class, "lr");
        rightRear = hardwareMap.get(DcMotorEx.class, "rr");
        rightFront = hardwareMap.get(DcMotorEx.class, "rf");
        //Carousel & Arm
        linSlide = hardwareMap.get(DcMotor.class, "linx");
        inCar = hardwareMap.get(DcMotor.class, "carin");
        box = hardwareMap.get(Servo.class, "hopper");
        colorDistance = hardwareMap.get(ColorSensor.class, "colorDistance");
        //Reversing inverted motors
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addLine("Ready to Deploy Teleop");
        telemetry.update();
    }
    @Override
    public void loop() {
        telemetry.clearAll();
        telemetry.addLine("Running Teleop");
        telemetry.addData("linSlide Encoder", linSlide.getCurrentPosition());
        if (colorDistance instanceof DistanceSensor) {
            telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorDistance).getDistance(DistanceUnit.CM));
            boxDist = ((DistanceSensor) colorDistance).getDistance(DistanceUnit.CM);
        }
        telemetry.update();

        //Drive Motor Powers
        leftFrontPower = gamepad1.right_stick_x*.8;
        leftRearPower = gamepad1.left_stick_y*.8;
        rightFrontPower = gamepad1.left_stick_y*.8;
        rightRearPower = gamepad1.right_stick_x*.8;

        if (gamepad1.left_trigger > 0.1) {
            leftFrontPower = -0.8 * gamepad1.left_trigger;
            rightRearPower = 0.8 * gamepad1.left_trigger;
        }
        if (gamepad1.right_trigger > 0.1) {
            leftFrontPower = 0.8 * gamepad1.right_trigger;
            rightRearPower = -0.8 * gamepad1.right_trigger;
        }

        //Arm and Carousel

        if (gamepad2.left_trigger > 0.05){
            inCarPow = 0.8;
            boxPos = 0;
        }
        else if (gamepad2.right_trigger > 0.05){
            inCarPow = -0.8;
        }
        else if (gamepad2.a){
            boxPos = .95;
        }
        else{
            boxPos = .25;
            inCarPow = 0;
        }

        if (boxDist < 5){
            freight = true;
        }

        if (freight){
            if (gamepad2.a){
                boxPos = .8;
                freight = false;
            }
            else{
                boxPos = .25;
            }
        }


        //Set power variables to hardware
        armPow = gamepad2.left_stick_y*0.5;
        box.setPosition(boxPos);

        inCar.setPower(inCarPow);
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftRear.setPower(leftRearPower);
        rightRear.setPower(rightRearPower);
        linSlide.setPower(armPow);
    }
}