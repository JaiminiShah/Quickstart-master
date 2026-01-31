package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "LEDLight", group = "Test")
public class LEDLight extends OpMode {

    DigitalChannel led01;
    DigitalChannel led23;

    @Override
    public void init() {
        led01 = hardwareMap.get(DigitalChannel.class, "led01");
        led23 = hardwareMap.get(DigitalChannel.class, "led23");

        led01.setMode(DigitalChannel.Mode.OUTPUT);
        led23.setMode(DigitalChannel.Mode.OUTPUT);

        // OFF at init
        led01.setState(true);
        led23.setState(true);
    }

    @Override
    public void loop() {

        // Gamepad button control
        if (gamepad1.a) {
            led01.setState(false); // LED ON
        } else {
            led01.setState(true);  // LED OFF
        }

        if (gamepad1.b) {
            led23.setState(false); // LED ON
        } else {
            led23.setState(true);  // LED OFF
        }
    }
}
