package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DigitalButton {
    private boolean previous, current;
    private DigitalChannel input;

    public DigitalButton(){
        previous = false;
        current = false;
    }
    public DigitalButton(HardwareMap hardwareMap, String name){
        input = hardwareMap.get(DigitalChannel.class, "name");
        input.setMode(DigitalChannel.Mode.INPUT);

        previous = false;
        current = input.getState();
    }

    public void update(boolean value){
        previous = current;
        current = value;
    }

    public void update(){
        previous = current;
        current = input.getState();
    }

    /**
     * Rising edge detector
     * @return true if the button was just pressed
     */
    public boolean wasJustPressed(){
        return current && !previous;
    }

    /**
     * Falling edge detector
     * @return true if the button was just released
     */
    public boolean wasJustReleased(){
        return !current && previous;
    }

    public boolean isDown(){
        return current;
    }
}
