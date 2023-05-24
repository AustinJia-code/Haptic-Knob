package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Knob", group = "Final")
public class KnobOpMode extends OpMode {
    HapticKnob knob;
    ModeChain modeChain;

    @Override
    public void init() {
        knob = new HapticKnob(hardwareMap);
        modeChain = new ModeChain();

        telemetry.addLine("Status: Initialized");
        telemetry.update();
    }

    @Override
    public void start(){}

    @Override
    public void loop() {
        knob.drive(modeChain.getMode());
        telemetry.addData("Current Pos: ", knob.getOutput());
    }
}