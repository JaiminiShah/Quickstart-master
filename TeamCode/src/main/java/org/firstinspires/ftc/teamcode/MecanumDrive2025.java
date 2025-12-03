package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MecanumDrive2025 {
     DcMotor frontLeftMotor;
     DcMotor frontRightMotor;
     DcMotor backLeftMotor;
     DcMotor backRightMotor;
     DcMotor mainIntake;
     DcMotorEx Launcher1;
     DcMotorEx Launcher2;
     final String TELEOP = "TELEOP";
     final String AUTO_BLUE = "AUTO BLUE";
     final String AUTO_RED = " AUTO RED";
     String operationSelected = TELEOP;
     double WHEELS_INCHES_TO_TICKS = (28 * 5 * 3) / (3 * Math.PI);
     ElapsedTime autoLaunchTimer = new ElapsedTime();
     ElapsedTime autoDriveTimer = new ElapsedTime();
     int bankVelocity = 1300;
     int farVelocity = 1900;
     int maxVelocity = 2200;

    public void init(HardwareMap hardwareMap) {
        frontLeftMotor = hardwareMap.dcMotor.get("front_left_motor");
        frontRightMotor = hardwareMap.dcMotor.get("front_right_motor");
        backLeftMotor = hardwareMap.dcMotor.get("back_left_motor");
        backRightMotor = hardwareMap.dcMotor.get("back_right_motor");
        mainIntake=hardwareMap.dcMotor.get("intake");
        Launcher1= (DcMotorEx) hardwareMap.dcMotor.get("launcher1");
        Launcher2=(DcMotorEx) hardwareMap.dcMotor.get("launcher2");

        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
         frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setPowers(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
         double maxSpeed = 1.0;
         maxSpeed = Math.max(maxSpeed, Math.abs(frontLeftPower));
         maxSpeed = Math.max(maxSpeed, Math.abs(frontRightPower));
         maxSpeed = Math.max(maxSpeed, Math.abs(backLeftPower));
         maxSpeed = Math.max(maxSpeed, Math.abs(backRightPower));

         frontLeftPower /= maxSpeed;
         frontRightPower /= maxSpeed;
         backLeftPower /= maxSpeed;
         backRightPower /= maxSpeed;

         frontLeftMotor.setPower(frontLeftPower);
         frontRightMotor.setPower(frontRightPower);
         backLeftMotor.setPower(backLeftPower);
         backRightMotor.setPower(backRightPower);
         }

         // Thanks to FTC16072 for sharing this code!!
         public void drive(double forward, double right, double rotate) {
            double frontLeftPower = forward + right + rotate;
            double frontRightPower = forward - right - rotate;
            double backLeftPower = forward - right + rotate;
            double backRightPower = forward + right - rotate;

         setPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
         }
 }


