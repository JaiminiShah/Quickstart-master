package org.firstinspires.ftc.teamcode.subSystem;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
//import dev.nextftc.hardware.impl.ServoEx;

@Config
public class shooter implements Subsystem {
    public static final shooter INSTANCE = new shooter();
    public static final MotorEx shooterMotor = new MotorEx("launcher1");
    public static final MotorEx shooterMotor1=new MotorEx("launcher2");
    //public static final ServoEx incline=new ServoEx("incline");
    public static double onTarget = 1700;
    public static double reversePower = -0.3;
    public static double tolerance = 40;

    public static Mode mode = Mode.OFF;
    public static double kP = 0.01;
    public static double kV = 0.00053;
    public static double kS = 0.075;
    public static boolean waitForSensor = false;

    private shooter() {
    }

    public static double getVelocity() {
        return -shooterMotor.getVelocity();
    }

    public static boolean upToSpeed() {
        return getVelocity() >= onTarget - tolerance;
    }

    public static Command shoot3() {
        return new SequentialGroup(
                shoot(),
                shoot(),
                shoot()
        );
    }

    private static Command shoot() {
        return new SequentialGroup(
                new WaitUntil(() -> (!waitForSensor || TransferDistanceSensor.hasBall()) && shooter.upToSpeed()),
                new Delay(0.1),
                Paddle.shoot()
        );
    }

    @Override
    public void periodic() {
        double target = 0;
        switch (mode) {
            case OFF:
                target = 0;
                break;
            case FORWARD:
                target = onTarget;
                break;
            case REVERSED:
                shooterMotor.setPower(-reversePower);
                break;
        }
        double currentVelocity = getVelocity();
        if (mode != Mode.REVERSED) {
            double power = kP * (target - currentVelocity) + kV * target + kS * Math.signum(target);
            shooterMotor.setPower(-power);
            FtcDashboard.getInstance().getTelemetry().addData("Shooter Target", target);
        }
        FtcDashboard.getInstance().getTelemetry().addData("Shooter Velocity", currentVelocity);
    }

    public enum Mode {
        OFF,
        FORWARD,
        REVERSED
    }
}