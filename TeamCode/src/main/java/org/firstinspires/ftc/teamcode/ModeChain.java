package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;

public class ModeChain {
    int index = 0;
    ArrayList<Mode> modeChain = new ArrayList<>();

    public ModeChain(){
        modeChain.add(Mode.FRICTIONLESS);   //0
        modeChain.add(Mode.FRICTION);       //1
        modeChain.add(Mode.DETENT);       //2
    }
    public Mode getMode() {
        return modeChain.get(index);
    }

    public void nextMode(){
        if(index < modeChain.size()-1) index++;
        else index = 0;
    }

    public void previousMode(){
        if(index > 0) index--;
        else index = modeChain.size()-1;
    }

    public void setMode(Mode mode){
        index = modeChain.indexOf(mode);
    }
}
