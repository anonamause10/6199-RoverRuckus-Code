package org.firstinspires.ftc.teamcode;

public class Phase {
    public int phase;
    public int cap;

    public Phase() {
        phase = 0;
        cap = 2;
    }

    public void setPhase(int capacity) {
        this.cap = capacity;
    }

    public int getPhase(){
        return phase;
    }

    public int getCap() {
        return cap;
    }

    public void next(){
        phase++;
        if(phase==cap){
            phase=0;
        }
    }

    public void back() {
        phase--;
        if (phase == -1) {
            phase = cap - 1;
        }
    }
}
