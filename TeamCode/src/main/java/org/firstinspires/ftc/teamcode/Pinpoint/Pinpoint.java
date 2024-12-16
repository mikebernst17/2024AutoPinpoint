package org.firstinspires.ftc.teamcode.Pinpoint;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.mmPerInch;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Pinpoint {

    // Odometry pods x and y offsets - MUST BE FILLED IN
    final static double XOFFSET = mmPerInch * 5.2;
    final static double YOFFSET = mmPerInch * 5.6;


    private GoBildaPinpointDriver odo;
    private DriveToPoint nav;

    public Pinpoint(LinearOpMode opMode, HardwareMap hwMap, Telemetry telemetry) {

        // Initialize the Pinpoint
        initPinpoint(hwMap);

        // Initiaize DriveToPoint
        nav = new DriveToPoint(opMode);
        nav.initializeMotors();
        nav.setXYCoefficients(0.01, 0, 2.0, MM, 12);
        nav.setYawCoefficients(2, 0.5, 2.0, DEGREES, 2);
    }

    public void initPinpoint(HardwareMap hwMap) {
        odo = hwMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(XOFFSET, YOFFSET);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
    }

    public boolean driveTo(Pose2D targetPosition, double power, double holdTime) {
        return nav.driveTo(odo.getPosition(), targetPosition, power, holdTime);

    }
    public void update() {
        odo.update();
    }

    public Pose2D getCurrentPosition() {
        return odo.getPosition();
    }

    public void resetPose() {odo.resetPosAndIMU();}
    public double getXpos() {
        return odo.getEncoderX();
    }
    public double getYpos() {
        return odo.getEncoderY();
    }
}