package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;

public class HapticKnob{
    private DcMotor knob;
    double output;
    int currentTick, previousTick;
    int offset;

    /**
     * Generates new Haptic Knob object:
     *  - Instantiates knob motor from control hub
     *  - Resets motor encoder
     *  - Sets mode to run based on power rather than the native velocity controller
     *  - Sets mode to not actively resist change in position when set to 0 power
     *
     * @param hardwareMap of control hub
     */
    public HapticKnob(HardwareMap hardwareMap){
        knob = hardwareMap.dcMotor.get("knob");
        knob.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        knob.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        knob.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        currentTick = knob.getCurrentPosition();
        output = 0.0;
        offset = 0;
    }

    /**
     * Sets offset equal to current knob position in ticks
     *
     * Offset is added to tick reading every loop to calculate "currentTick"
     */
    public void resetTicks(){
        offset = knob.getCurrentPosition();
    }

    /**
     * @return Knob's set power, [-1, 1]
     */
    public double getPower(){
        return knob.getPower();
    }

    /**
     * @return Output generated from various knob drive modes
     */
    public double getOutput(){
        return output;
    }

    /**
     * Calculates current tick
     * Runs motor power according to the drive mode parameter
     *
     * Set output equal to the drive mode's returned double
     *
     * @param mode Drive mode required of the knob
     * @param input User input, such as from a game pad, used for some drive modes, [-1, 1]
     * @see {@link org.firstinspires.ftc.teamcode.Mode}
     */
    public void drive(Mode mode, double input){
        previousTick = currentTick;
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

    /**
     * Definition for "friction" drive mode
     * Constants defined in {@link org.firstinspires.ftc.teamcode.Constants.Friction}
     *
     * Sets knob power to 0 when within it's defined range
     * Sets knob power to proportionally return to respective bound when out of it's defined range
     *
     * @return Knob position [-1, 1] along it's defined range
     */
    public double friction(){
        double position = (double) currentTick / Constants.Friction.RIGHT_BOUND_TICKS;

        if(Math.abs(position) <= 1){
            knob.setPower(0);
            return position;
        }

        double power = -1 * (position % 1) * Constants.Friction.K_P;
        knob.setPower(power);

        return Math.signum(currentTick);
    }

    /**
     * Definition for "frictionless" drive mode
     * Constants defined in {@link org.firstinspires.ftc.teamcode.Constants.Frictionless}
     *
     * Sets knob power to 0, ideally spinning freely
     *
     * @return Amount of times the knob has travelled over defined range, [-inf, inf]
     */
    public double frictionless() {
        //TODO: Cook up a good way to imitate frictionless on non-brushless
        knob.setPower(0);

        return currentTick / Constants.Frictionless.RANGE_TICKS;
    }

    /**
     * Definition for "detent" drive mode
     * Constants defined in {@link org.firstinspires.ftc.teamcode.Constants.Detent}
     *
     * Divides 360 degrees of movement into a defined amount of sections
     * Section borders are numbered starting with 0 at 0 degrees, increasing clockwise
     * Example: A detent of 12 sections would match an analog clock with the 12 replaced with 0, and the knob would seek to snap to the numbers
     * Powers motor towards the nearest section border, proportional to it's distance
     *
     * @return Section border the knob is nearest to, [0, {@link org.firstinspires.ftc.teamcode.Constants.Detent.SECTIONS} - 1]
     */
    public double detent(){
        double positionInSection = (double) (Math.abs(currentTick) % Constants.Detent.SECTION_RANGE_TICKS) / Constants.Detent.SECTION_RANGE_TICKS;
        double distanceToSnap =
                positionInSection < 0.5 ? -1 * Math.signum(currentTick) * positionInSection : Math.signum(currentTick) * (1 - positionInSection);

        knob.setPower(distanceToSnap * Constants.Detent.K_P);

        double temp = currentTick % Constants.Knob.TICKS;                   // get rotation of the knob, regardless of which revolution it's on
        if(temp < 0) temp += Constants.Knob.TICKS;                          // if the rotation was negative, normalize it to [0, Knob.TICKS]

        temp = Math.round(temp / Constants.Detent.SECTION_RANGE_TICKS);     // 
        if(temp > Constants.Detent.SECTIONS - 1){
            temp = 0;
        }

        return temp;
    }

    /**
     * Definition for "input" drive mode
     * Constants defined in {@link org.firstinspires.ftc.teamcode.Constants.Input}
     *
     * Calculates the knob's current position [-1, 1] along the defined range
     * Powers the knob toward the input proportional to the distance
     *
     * @param input From user, such as from a game pad, [-1, 1]
     * @return Knob's true position along defined range, [-1, 1]
     */
    public double input(double input){
        double targetTick = input * Constants.Input.RIGHT_BOUND_TICKS;
        knob.setPower((targetTick - currentTick) / Constants.Input.RANGE_TICKS * Constants.Input.K_P);

        return currentTick / Constants.Input.RIGHT_BOUND_TICKS;
    }
}
