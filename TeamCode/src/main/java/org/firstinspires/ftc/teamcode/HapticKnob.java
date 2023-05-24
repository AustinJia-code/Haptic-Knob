package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;

public class HapticKnob{
    private DcMotor knob;
    double output;
    int currentTick;
    int offset;

    public HapticKnob(HardwareMap hardwareMap){
        knob = hardwareMap.dcMotor.get("knob");
        knob.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        knob.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        knob.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        currentTick = knob.getCurrentPosition();
        output = 0.0;
        offset = 0;
    }

    public void resetTicks(){
        offset = knob.getCurrentPosition();
    }

    public double getOutput(){
        return output;
    }

    public void drive(Mode mode, double input){
        currentTick = knob.getCurrentPosition() - offset;
        switch(mode){
            case FRICTION:
                output = friction();
                break;
            case FRICTIONLESS:
                output = frictionless();
                break;
            case DETENT:
                output = detent();
                break;
            case INPUT:
                output = input(input);
                break;
        }
    }

    public double friction(){
        int difference = Math.abs(currentTick) - Constants.Friction.RIGHT_BOUND;

        if(difference < 0){
            knob.setPower(0);
            return currentTick - Constants.Friction.RIGHT_BOUND;
        }

        double power = difference * Constants.Friction.K_P;
        if(currentTick < 0){
            knob.setPower(1 * power);
            return -1;
        }else{
            knob.setPower(-1 * power);
            return 1;
        }
    }

    //TODO: Cook up a good way to imitate frictionless on non-brushless
    public double frictionless() {
        knob.setPower(0);

        return currentTick / Constants.FRICTIONLESS.RANGE_TICKS;
    }

    public double detent(){
        double positionInSection = (double) (Math.abs(currentTick) % Constants.Detent.SECTION_RANGE_TICKS) / Constants.Detent.SECTION_RANGE_TICKS;

        double distanceToSnap =
                positionInSection < 0.5 ? -1 * Math.signum(currentTick) * positionInSection : Math.signum(currentTick) * (1 - positionInSection);
        knob.setPower(distanceToSnap * Constants.Detent.K_P);

        double temp = currentTick % Constants.Knob.TICKS;
        if(temp < 0) temp += Constants.Knob.TICKS;
        else if(temp > Constants.Knob.TICKS) temp -= Constants.Knob.TICKS;

        temp = Math.round(temp / Constants.Detent.SECTION_RANGE_TICKS);
        if(temp > Constants.Detent.SECTIONS - 1){
            temp = 0;
        }

        return temp;
    }

    public double input(double input){
        double targetTick = input * Constants.Input.RIGHT_BOUND_TICKS;
        knob.setPower((targetTick - currentTick) / Constants.Input.RANGE_TICKS * Constants.Input.K_P);

        return currentTick / Constants.Input.RIGHT_BOUND_TICKS;
    }
}
