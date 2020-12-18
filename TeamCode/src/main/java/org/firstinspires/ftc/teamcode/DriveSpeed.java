package org.firstinspires.ftc.teamcode;

public enum DriveSpeed {
    LOWPOWER(.25), MIDPOWER(.5), HIGHPOWER(.75);

    private double speed;

    private DriveSpeed(double speed){
        this.speed = speed;
    }

    double getSpeed(){
        return speed;
    }
}

