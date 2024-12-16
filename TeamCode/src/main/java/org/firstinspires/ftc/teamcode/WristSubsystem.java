// a subsystem to control NOLAN's rotating wrist thing.
// This is used on the 28147 bot ONLY as of 12/4/2024
// The 16760 bot uses BRENDAN's wrist and claw.
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class WristSubsystem {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    // servo motors:
    private Servo close;
    private Servo wrist;

    // important values
    private boolean wristDown = true;
    private boolean clawClosed = true;

    public WristSubsystem(HardwareMap hwmap, Telemetry tele){
        // init important things
        this.hardwareMap = hwmap;
        this.telemetry = tele;

        // init motors
        this.close = hwmap.get(Servo.class, "RightFinger");
        this.wrist = hwmap.get(Servo.class, "ShoulderPivot");

        // scale servo ranges
        this.close.scaleRange(0.4,0.7);
        this.wrist.scaleRange(0.35,0.7);
    }

    public void wristUp(){
        // move the wrist to the side
        this.wrist.setPosition(1);
        this.wristDown = false;
    }

    public void wristDown(){
        // move the wrist to a downward position
        this.wrist.setPosition(0);
        this.wristDown = true;
    }

    public void clawClose(){
        // move the claw closed
        this.close.setPosition(0);
        this.clawClosed = true;
    }

    public void clawOpen(){
        // open the claw
        this.close.setPosition(1);
        this.clawClosed = false;
    }

    public boolean toggleWrist(){
        // opens the wrist if it is closed.
        // closes the wrist if it is open.
        // returns the new state of the wrist

        if (!this.wristDown){
            // wrist is up. Move it down.
            this.wristDown();
        }
        else{
            // wrist is not up. thus, it is down. Move it up.
            this.wristUp();
        }

        // return the new state
        return this.wristDown;
    }

    public boolean toggleClaw(){
        // openss the claw if it is closed
        // closes it if it s open.
        // return the new state
        // (same thing as teh other one)

        if (!this.clawClosed){
            // claw is open. close it
            this.clawClose();
        }
        else{
            // claw is not open. it wants to be
            this.clawOpen();
        }

        //returns current state
        return this.clawClosed;
    }
}
