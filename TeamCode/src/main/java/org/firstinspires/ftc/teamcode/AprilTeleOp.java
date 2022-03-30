package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp(name = "April TeleOp")
public class AprilTeleOp extends OpMode {
    public DcMotor leftFront, leftRear, rightRear, rightFront;
    public DcMotorEx carin, linx;
    public Servo hopper;
    double leftFrontPower, rightFrontPower, leftRearPower, rightRearPower;

    //initializes all the aspects we need to make our robot function
    public double armPosScale = 50, armPowScale = 0.0015;
    public double armPosCurrent=0, armPosDes=0, armPosError=0, armPow=0;
    public double integrater = 0.001, intpower = 0.00075, multiplier = 1, speedK = 1;

    @Override
    public void init() {
        telemetry.clearAll();
        // defining all the hardware
        //Wheels
        leftFront = hardwareMap.get(DcMotorEx.class, "lf");
        leftRear = hardwareMap.get(DcMotorEx.class, "lr");
        rightRear = hardwareMap.get(DcMotorEx.class, "rr");
        rightFront = hardwareMap.get(DcMotorEx.class, "rf");
        // PTO
        carin = hardwareMap.get(DcMotorEx.class, "carin");
        //Arm
        linx = hardwareMap.get(DcMotorEx.class, "linx");
        hopper = hardwareMap.get(Servo.class, "hopper");
        //Reversing inverted motors
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addLine("Ready to Deploy Teleop");
        telemetry.update();
    }
    @Override
    public void loop() {
        telemetry.clearAll();
        telemetry.addLine("Running Teleop");
        telemetry.update();

        // Assign motor variable powers
        leftFrontPower = -gamepad1.right_stick_x;
        leftRearPower = -gamepad1.left_stick_y;
        rightFrontPower = -gamepad1.left_stick_y;
        rightRearPower = -gamepad1.right_stick_x;


        if (gamepad1.right_trigger > 0.1) {
            leftFrontPower = -0.75 * gamepad1.left_trigger;
            rightRearPower = 0.75 * gamepad1.left_trigger;
        }
        if (gamepad1.left_trigger > 0.1) {
            leftFrontPower = 0.75 * gamepad1.right_trigger;
            rightRearPower = -0.75 * gamepad1.right_trigger;
        }

        // Brake motors when not used
        if (gamepad1.left_stick_y <= 0.1 && gamepad1.left_trigger <= 0.1 && gamepad1.right_trigger <= 0.1) {
            rightFront.setPower(0);
            leftRear.setPower(0);
        }

        // Brake motors when not used
        if (gamepad1.right_stick_x <= 0.1 && gamepad1.left_trigger <= 0.1 && gamepad1.right_trigger <= 0.1) {
            leftFront.setPower(0);
            rightRear.setPower(0);
        }

        leftRear.setPower(leftRearPower);
        leftFront.setPower(leftFrontPower);
        rightRear.setPower(rightRearPower);
        rightFront.setPower(rightFrontPower);

        if(gamepad2.dpad_up == true) {
            hopper.setPosition(0.6);

         }else if(gamepad2.dpad_left == true || gamepad2.dpad_right == true) {
            hopper.setPosition(0.8);

        }else if(gamepad2.dpad_down == true) {
            hopper.setPosition(1.0);

        } else if(gamepad2.b == true) {
            hopper.setPosition(0);
            carin.setPower(1);

        } else if(gamepad2.x == true) {
            hopper.setPosition(0);
            carin.setPower(-1);

        } else if(gamepad2.right_trigger > 0) {
            carin.setPower(gamepad2.right_trigger*0.5);

        } else if(gamepad2.left_trigger > 0) {
            carin.setPower(-gamepad2.left_trigger);

        } else {
            hopper.setPosition(0.25);
            carin.setPower(0);
        }
/*
        //Get the encoder position for the arm
        armPosCurrent = linx.getCurrentPosition();
        //Pure Proportional Feedback
        armPosDes += speedK*armPosScale*gamepad2.right_stick_y;
        //input scale factor
        armPosError = armPosDes - armPosCurrent;
        //integrater += armPosError;                                           //unnecessary
        armPow = Math.min(Math.max(armPowScale*armPosError, -1.00), 1.00);
        //proportional gain
        if(armPow >= 1){ armPosDes = armPosCurrent+(1/armPowScale); }       //AntiWindup Code
        if(armPow <= -1) {armPosDes = armPosCurrent-(1/armPowScale); }      //AntiWindup Code
        //Accelerate arms set motor based on proximity to desired position
        linx.setPower(armPow);

 */
        linx.setPower(0.4);
    }

    @Override
    public void stop() {
    }
}