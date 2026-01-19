package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class FlywheelAuto {
    private Servo gateServo;
    private CRServo flywheelServo;
    private ElapsedTime stateTimer=new ElapsedTime();
    private enum FlywheelState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        RESET_GATE
    }
    




}
