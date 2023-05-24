package org.firstinspires.ftc.teamcode;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Knob", group = "Final")
public class KnobOpMode extends OpMode {
    HapticKnob knob;
    ModeChain modeChain;
    int input;
    GamepadEx gamepad;

    @Override
    public void init() {
        knob = new HapticKnob(hardwareMap);
        modeChain = new ModeChain();
        input = 0;
        gamepad = new GamepadEx(gamepad1);

        telemetry.addLine("Status: Initialized");
        telemetry.update();
    }

    @Override
    public void start(){}

    @Override
    public void loop() {
        if(gamepad.wasJustPressed(RIGHT_BUMPER)){
            knob.resetTicks();
            modeChain.nextMode();
        }
        if(gamepad.wasJustPressed(LEFT_BUMPER)){
            knob.resetTicks();
            modeChain.previousMode();
        }

        knob.drive(modeChain.getMode(), gamepad.getRightX());

        telemetry.addData("Mode: ", modeChain.getMode());
        telemetry.addData("Current Pos: ", knob.getOutput());
    }
}