package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
 import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


 @TeleOp()
 public class GG_TankDrive extends OpMode {
          MecanumDrive2025 drive = new MecanumDrive2025();

         @Override
        public void init() {
         drive.init(hardwareMap);
         }
          private void setPowers(double frontLeftPower, double frontRightPower, double
             backLeftPower, double backRightPower) {
          double maxSpeed = 1.0;
          maxSpeed = Math.max(maxSpeed, Math.abs(frontLeftPower));
          maxSpeed = Math.max(maxSpeed, Math.abs(frontRightPower));
          maxSpeed = Math.max(maxSpeed, Math.abs(backLeftPower));
          maxSpeed = Math.max(maxSpeed, Math.abs(backRightPower));

          frontLeftPower /= maxSpeed;
          frontRightPower /= maxSpeed;
          backLeftPower /= maxSpeed;
          backRightPower /= maxSpeed;

         drive.frontLeftMotor.setPower(frontLeftPower);
         drive.frontRightMotor.setPower(frontRightPower);
         drive. backLeftMotor.setPower(backLeftPower);
         drive. backRightMotor.setPower(backRightPower);
          }


        private void drive(double forward, double right, double rotate) {
              double frontLeftPower = forward + right + rotate;
              double frontRightPower = forward - right - rotate;
              double backLeftPower = forward - right + rotate;
              double backRightPower = forward + right - rotate;
              setPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
          }

     @Override
       public void loop() {
           double forward = -gamepad1.left_stick_y;
           double right = gamepad1.left_stick_x;
           double rotate = gamepad1.right_stick_x;

           drive.drive(forward, right, rotate);
         }
 }