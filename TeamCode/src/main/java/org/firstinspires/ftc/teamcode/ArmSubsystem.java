package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmSubsystem {

    private DcMotor Elevation = null;
    private DcMotor Extension = null;

    // here is the telemetry:
    private Telemetry telemetry;

    private int speed;
    private int elevationTarget;
    private int extensionTarget;
    // Constructor
    public ArmSubsystem(HardwareMap hwMap, Telemetry telemetry) {

        //set telemetry:
        this.telemetry = telemetry;
        this.speed = -1;
        //actually define elevate and extend
        Elevation = hwMap.get(DcMotor.class, "Elevation");
        Extension = hwMap.get(DcMotor.class, "Extension");
        Elevation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //set modes to run using the encoder
//        Elevation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Elevation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        Extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Extension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    //    You have to call setElevationPosition before calling armUp or armDown.
//    If you are calling armUp set it to a negative and armDown a positive.
//    Make sure to check if it the first time calling this (see ArmTestTeleop.java)
//    Basically NEVER run this in a loop or i will kill you if you come to me asking why it doesn't work
    public void setElevationTarget(int newPosition){
        elevationTarget = Elevation.getCurrentPosition() + newPosition;
    }

    //same as setElevationTarget. Extend is negative
    public void setExtensionTarget(int newPosition){
        extensionTarget = Extension.getCurrentPosition() + newPosition;
    }
    // Move arm up until the newPosition reached
    public boolean armUp(double speed) {
        telemetry.addData("arm pos: ", Elevation.getCurrentPosition());
        telemetry.addData("target arm pos: ", elevationTarget);
        Elevation.setPower(-speed);
        if (Elevation.getCurrentPosition()>=elevationTarget){
            return true;
        }
        Elevation.setPower(0);
        return false;
    }

    // Move arm down until newPosition reached
    public boolean armDown(double speed) {
        telemetry.addData("arm pos: ", Elevation.getCurrentPosition());
        telemetry.addData("target arm pos: ", elevationTarget);
        Elevation.setPower(speed);
        if (Elevation.getCurrentPosition()<=elevationTarget){
            return true;
        }

        Elevation.setPower(0);
        return false;
    }

    // Extend arm out until the newPosition reached
    public boolean armExtend(double speed) {
        telemetry.addData("current extension pos: ", Extension.getCurrentPosition());
        telemetry.addData("target extension pos: ", extensionTarget);
        Extension.setPower(speed);
        if (Extension.getCurrentPosition()<=extensionTarget){
            telemetry.addData("reached position", 1);
            return true;
        }
        Extension.setPower(0);

        return false;
    }

    // Extend arm out until the newPosition reached
    public boolean armRetract(double speed) {
        telemetry.addData("current extension pos: ", Extension.getCurrentPosition());
        telemetry.addData("target extension pos: ", extensionTarget);
        Extension.setPower(-speed);
        if (Extension.getCurrentPosition()>=extensionTarget){
            return true;
        }
        Extension.setPower(0);


        return false;
    }
    public int getElevationPos () {
        return Elevation.getCurrentPosition();
    }
    public int getExtensionPos () {
        return Extension.getCurrentPosition();
    }
}
