package org.firstinspires.ftc.teamcode;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Knob", group = "Final")
public class KnobOpMode extends OpMode {
    HapticKnob knob;
    ModeChain modeChain;
    int input, loops;
    GamepadEx gamepad;

    @Override
    public void init() {
        knob = new HapticKnob(hardwareMap);
        modeChain = new ModeChain();
        input = loops = 0;
        gamepad = new GamepadEx(gamepad1);

        telemetry.addLine("Status: Initialized");
        telemetry.update();
    }

    @Override
    public void start(){
        knob.setMode(Mode.OUTPUT);
    }

    @Override
    public void loop() {
        loops++;

        if(gamepad.wasJustPressed(RIGHT_BUMPER)){
            knob.resetTicks();
            knob.setMode(modeChain.nextMode());
        }
        if(gamepad.wasJustPressed(LEFT_BUMPER)){
            knob.resetTicks();
            knob.setMode(modeChain.previousMode());
        }

        //knob.drive(gamepad.getRightX());

        knob.drive(Math.sin(Math.toRadians(loops/360.0)));

        telemetry.addData("Mode: ", modeChain.getMode());
        telemetry.addData("Current Pos: ", knob.getOutput());
        telemetry.addData("Power: ", knob.getPower());
    }
}