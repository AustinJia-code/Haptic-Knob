package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Mode.FRICTION;

import com.qualcomm.robotcore.hardware.*;

public class HapticKnob{
    private DcMotorEx knob;
    double output;
    int currentTick, previousTick;
    double previousLoopStartMS, currentLoopStartMS, looptimeMs;
    int offset;
    double previousKnobVelocity, knobVelocity;
    Mode mode;

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
        knob = hardwareMap.get(DcMotorEx.class, "Knob");
        knob.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setMode(FRICTION);

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
     * @return loop time of drive method in milliseconds
     */
    public double getLooptimeMs(){return looptimeMs;}

    /**
     * Calculates current tick
     * Runs motor power according to the drive mode parameter
     *
     * Set output equal to the drive mode's returned double
     *
     * @param input User input, such as from a game pad, used for some drive modes, [-1, 1]
     * @see {@link org.firstinspires.ftc.teamcode.Mode}
     */
    public void drive(double input){
        previousTick = currentTick;
        currentTick = knob.getCurrentPosition() - offset;

        previousLoopStartMS = currentLoopStartMS;
        currentLoopStartMS = System.currentTimeMillis();

        looptimeMs = currentLoopStartMS - previousLoopStartMS;

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
            case OUTPUT:
                output = output(input);
                break;
        }
    }

    public void setMode(Mode mode){
        this.mode = mode;

        switch(mode){
            case OUTPUT:
                knob.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                knob.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                break;
            default:
                knob.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                knob.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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
     * Sets knob power to match user spin speed, ideally simulating a knob spinning freely
     *  - Find knob acceleration
     *  - Adjust knob power to mirror human input
     *
     * @return Amount of times the knob has travelled over defined range, [-inf, inf]
     */
    public double frictionless() {
        previousKnobVelocity = knobVelocity;
        knobVelocity = (currentTick - previousTick) / looptimeMs;    //Expressed in ticks per millisecond
        double acceleration = knobVelocity - previousKnobVelocity;

        double power = knob.getPower() + (acceleration / Constants.Frictionless.MAX_VELOCITY);

        knob.setPower(power);

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

        double position = currentTick % Constants.Knob.TICKS;                   // get rotation of the knob, regardless of which revolution it's on
        if(position < 0) position += Constants.Knob.TICKS;                          // if the rotation was negative, normalize it to [0, Knob.TICKS]

        position = Math.round(position / Constants.Detent.SECTION_RANGE_TICKS);
        if(position > Constants.Detent.SECTIONS - 1){
            position = 0;
        }

        return position;
    }

    /**
     * Definition for "output" drive mode
     * Constants defined in {@link Constants.Output}
     *
     * Calculates the knob's current position [-1, 1] along the defined range
     * Powers the knob toward the input proportional to the distance, "outputting" a visualizer
     *
     * @param input From user, such as from a game pad, [-1, 1]
     * @return Knob's true position along defined range, [-1, 1]
     */
    public double output(double input){
        double targetTick = input * Constants.Output.RIGHT_BOUND_TICKS;
        knob.setPower((targetTick - currentTick) / Constants.Output.RANGE_TICKS * Constants.Output.K_P);

        return currentTick / Constants.Output.RIGHT_BOUND_TICKS;
    }
}
