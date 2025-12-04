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

         @Override
       public void loop() {
           double forward = -gamepad1.left_stick_y;
           double right = gamepad1.left_stick_x;
           double rotate = gamepad1.right_stick_x;

           drive.drive(forward, right, rotate);
         }
 }