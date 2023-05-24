package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;

public class HapticKnob{
    private DcMotor knob;
    double output;

    public HapticKnob(HardwareMap hardwareMap){
        knob = hardwareMap.dcMotor.get("knob");
        knob.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        knob.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        output = 0.0;
    }

    public double getOutput(){
        return output;
    }

    public void drive(Mode mode){
        switch(mode){
            case FRICTION:
                friction();
                break;
            case FRICTIONLESS:
                frictionless();
                break;
            case ROLLOVER:
                rollover();
                break;
        }
    }

    public void friction(){

    }
    public void frictionless() {
    }

    public void rollover(){

    }
}
