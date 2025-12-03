package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.MecanumDrive2025.*;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp()
  class GGTeleop_2025 extends OpMode {
     MecanumDrive2025 drive = new MecanumDrive2025();
     IMU imu;
    // Setting our velocity targets. These values are in ticks per second!
  /*  private static final int bankVelocity = 1300;
    private static final int farVelocity = 1900;
    private static final int maxVelocity = 2200;*/

     @Override
     public void init() {
         drive.init(hardwareMap);

         imu = hardwareMap.get(IMU.class, "imu");
         RevHubOrientationOnRobot revHubOrientationOnRobot =
                 new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                 RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
                 imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

         }

         public void driveFieldRelative(double forward, double right, double rotate) {
         double robotAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
         // convert to polar
         double theta = Math.atan2(forward, right);
         double r = Math.hypot(forward, right);
         // rotate angle
         theta = AngleUnit.normalizeRadians(theta - robotAngle);
         // convert back to cartesian
         double newForward = r * Math.sin(theta);
         double newRight = r * Math.cos(theta);
         drive.drive(newForward, newRight, rotate);
         }
    public void manualCoreHexAndServoControl() {
        // Manual control for the Core Hex intake
        if (gamepad1.cross) {
            drive.mainIntake.setPower(0.5);
        } else if (gamepad1.triangle) {
            drive.mainIntake.setPower(-0.5);
        }
      /*  // Manual control for the hopper's servo
        if (gamepad1.dpad_left) {
            dservo.setPower(1);
        } else if (gamepad1.dpad_right) {
            servo.setPower(-1);
        } */
    }
    /**
     * This if/else statement contains the controls for the flywheel, both manual and auto.
     * Circle and Square will spin up ONLY the flywheel to the target velocity set.
     * The bumpers will activate the flywheel, Core Hex feeder, and servo to cycle a series of balls.
     */
    public void setFlywheelVelocity() {
        if (gamepad2.options) {
            drive.Launcher1.setPower(-0.5);
            drive.Launcher2.setPower(-0.5);
        } else if (gamepad2.left_bumper) {
            farPowerAuto();
        } else if (gamepad2.right_bumper) {
            bankShotAuto();
        } else if (gamepad2.circle) {
            ((DcMotorEx) drive.Launcher1).setVelocity(drive.bankVelocity);
            ((DcMotorEx) drive.Launcher2).setVelocity(drive.bankVelocity);

        } else if (gamepad2.square) {
            ((DcMotorEx) drive.Launcher1).setVelocity(drive.maxVelocity);
            ((DcMotorEx) drive.Launcher2).setVelocity(drive.maxVelocity);

        } else {
            ((DcMotorEx) drive.Launcher1).setVelocity(0);
            ((DcMotorEx) drive.Launcher1).setVelocity(0);
            drive.mainIntake.setPower(0);
            // The check below is in place to prevent stuttering with the servo. It checks if the servo is under manual control!
           /* if (!gamepad1.dpad_right && !gamepad1.dpad_left) {
                servo.setPower(0);
            }*/
        }
    }
    /**
     * The bank shot or near velocity is intended for launching balls touching or a few inches from the goal.
     * When running this function, the flywheel will spin up and the Core Hex will wait before balls can be fed.
     * The servo will spin until the bumper is released.
     */
    public void bankShotAuto() {
        ((DcMotorEx) drive.Launcher1).setVelocity(drive.bankVelocity);
        ((DcMotorEx) drive.Launcher2).setVelocity(drive.bankVelocity);
        //servo.setPower(-1);
        if ((((DcMotorEx) drive.Launcher1).getVelocity() >= drive.bankVelocity - 50) && (((DcMotorEx)drive.Launcher2).getVelocity()>=drive.bankVelocity-50)) {
            drive.mainIntake.setPower(1);
        } else {
            drive.mainIntake.setPower(0);
        }
    }
    /**
     * The far power velocity is intended for launching balls a few feet from the goal. It may require adjusting the deflector.
     * When running this function, the flywheel will spin up and the Core Hex will wait before balls can be fed.
     * The servo will spin until the bumper is released.
     */
    public void farPowerAuto() {
        ((DcMotorEx) drive.Launcher1).setVelocity(drive.farVelocity);
        ((DcMotorEx)drive.Launcher2).setVelocity(drive.farVelocity);
        //servo.setPower(-1);
        if ((((DcMotorEx) drive.Launcher1).getVelocity() >= drive.farVelocity - 100) && (((DcMotorEx) drive.Launcher2).getVelocity() >= drive.farVelocity - 100)){
            drive.mainIntake.setPower(1);
        } else {
            drive.mainIntake.setPower(0);
        }
    }

 public void loop() {
         double forward = -gamepad1.left_stick_y;
         double right = gamepad1.left_stick_x;
         double rotate = gamepad1.right_stick_x;
         driveFieldRelative(forward, right, rotate);
         }
}
