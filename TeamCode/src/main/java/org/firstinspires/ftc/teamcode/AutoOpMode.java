package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


import java.util.Timer;

@Autonomous(name="AutoOpMode")
//@Disabled

public class AutoOpMode extends LinearOpMode {

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    private final DriveToPoint nav = new DriveToPoint(this); //OpMode member for the point-to-point navigation class
    private DcMotor rotateArmMotor = null;
    private DcMotor extendArmMotor = null;
    private final ElapsedTime timer = new ElapsedTime();



    // Auto State Machine
    enum StateMachine {
        WAITING_FOR_START,
        DRIVE_TO_SUBMERSIBLE,
        PLACE_SPECIMEN,
        RELEASE_SPECIMEN,
        DRIVE_TO_OBSERVATION_ZONE,
        END
    }

    // Field poses
    static final Pose2D SUBMERSIBLE_POS = new Pose2D(DistanceUnit.INCH, -48, 12, AngleUnit.DEGREES, 0);
    static final Pose2D OBSERVATION_ZONE = new Pose2D(DistanceUnit.INCH,0,36, AngleUnit.DEGREES,90);

    final double DRIVE_SPEED = 0.3;
    final double ARM_POWER = -1;
    final double EXTEND_POWER = 1;
    final int ARM_UP_POSITION = -2800;
    final int ARM_DOWN_POSITION = 0;
    final int EXT_OUT_POSITION = 7500;
    final int EXT_IN_POSITION = 0;

    StateMachine stateMachine;
    boolean firstTime = true;
    int armTargetRotPos;
    int armTargetExtPos;

    @Override
    public void runOpMode() {
        Timer t;

        // Initialize the Pinpoint
        initPinpoint();

        // Initialize DriveToPoint
        nav.initializeMotors();
        nav.setXYCoefficients(0.01,0,2.0,DistanceUnit.INCH,.5);
        nav.setYawCoefficients(1,0.5,2.0, AngleUnit.DEGREES,6);

        // Initialize motors and servos
        rotateArmMotor = hardwareMap.get(DcMotor.class, "Elevation");
        rotateArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendArmMotor = hardwareMap.get(DcMotor.class, "Extension");
        extendArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rotateArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotateArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        // Start state machine
        stateMachine = StateMachine.WAITING_FOR_START;

        // Print current state
        displayDebugInfo();
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();

        //telemetry.setAutoClear(false);

        while (opModeIsActive()) {

            // Pinpoint update must be called every cycle
            odo.update();

            displayDebugInfo();

            //----------------------------------------------------------
            // State: WAITING_FOR_START
            //----------------------------------------------------------
            if (stateMachine == StateMachine.WAITING_FOR_START){
                stateMachine = StateMachine.DRIVE_TO_SUBMERSIBLE;
            }

            //----------------------------------------------------------
            // State: DRIVE_TO_SUBMERSIBLE
            // Actions: rotate arm
            //          extend arm
            //----------------------------------------------------------
            if (stateMachine == StateMachine.DRIVE_TO_SUBMERSIBLE) {
                // In parallel:
                // (a) drive to front of SUBMERSIBLE
                // (b) rotate arm to ARM_UP_MAX_POSITION
                // (c) extend arm to EXT_MAX_POSITION
                // When all three conditions met, move to next state (PLACE_SPECIMEN)
                boolean cond1 = nav.driveTo(odo.getPosition(), SUBMERSIBLE_POS, DRIVE_SPEED, 2);
                if (firstTime) {
                    armTargetRotPos = ARM_UP_POSITION;
                    armTargetExtPos = EXT_OUT_POSITION;
                    firstTime = false;
                }
                boolean cond2 = armUp(armTargetRotPos);
                boolean cond3 = armExtend(armTargetExtPos);
                if (cond1 && cond2 && cond3) {
                    stateMachine = StateMachine.PLACE_SPECIMEN;
                    firstTime = true;
                }
            }

            //----------------------------------------------------------
            // State: PLACE_SPECIMEN
            // Actions: retract arm until target location reached
            //----------------------------------------------------------
            if (stateMachine == StateMachine.PLACE_SPECIMEN){
                //boolean cond = armExtend(EXT_MAX_POSITION);
                //boolean cond = armRetract(EXT_PLACE_POSITION);
                //if (cond) {
                    stateMachine = StateMachine.RELEASE_SPECIMEN;
                //}
            }

            //----------------------------------------------------------
            // State: RELEASE_SPECIMEN
            // Actions: open fingers to release specimen
            //----------------------------------------------------------
            if (stateMachine == StateMachine.RELEASE_SPECIMEN){
                // Release specimen
                // TODO
                stateMachine = StateMachine.DRIVE_TO_OBSERVATION_ZONE;
            }

            //----------------------------------------------------------
            // State: DRIVE_TO_OBSERVATION_ZONE
            // Actions: drive to observation zone and park
            //----------------------------------------------------------
            if (stateMachine == StateMachine.DRIVE_TO_OBSERVATION_ZONE) {
                // Drive to observation zone
                boolean cond1 = nav.driveTo(odo.getPosition(), OBSERVATION_ZONE, DRIVE_SPEED, 0);
                if (firstTime) {
                    armTargetRotPos = ARM_DOWN_POSITION;
                    armTargetExtPos = EXT_IN_POSITION;
                    firstTime = false;
                }
                boolean cond2 = armDown(armTargetRotPos);
                boolean cond3 = armRetract(armTargetExtPos);
                if (cond1) {
                    stateMachine = StateMachine.END;
                    firstTime = true;
                }
            }

            //----------------------------------------------------------
            // State: END
            // Actions: drive to observation zone and park
            //----------------------------------------------------------
            if (stateMachine == StateMachine.END) {
                displayDebugInfo();
            }
        }
    }


    // Move arm up until the newPosition reached
    private boolean armUp(int newPosition) {
        boolean done = false;
        if (rotateArmMotor.getCurrentPosition() > newPosition) {
            // raise arm
            rotateArmMotor.setPower(ARM_POWER);
        }
        else { // arm is <= newPosition so stop motor
            rotateArmMotor.setPower(0);
            done = true;
        }
        return done;
    }

    // Move arm down until newPosition reached
    private boolean armDown(int newPosition) {
        boolean done = false;
        if (rotateArmMotor.getCurrentPosition() < newPosition) {
            // lower arm
            rotateArmMotor.setPower(-ARM_POWER);
        }
        else { //arm is >= newPosition so stop motor
            rotateArmMotor.setPower(0);
            done = true;
        }
        return done;
    }

    // Extend arm out until the newPosition reached
    private boolean armExtend(int newPosition) {
        boolean done = false;
        if (extendArmMotor.getCurrentPosition() < newPosition) {
            // extend arm
            extendArmMotor.setPower(EXTEND_POWER);
        }
        else { //extension is >= newPosition so stop motor
            extendArmMotor.setPower(0);
            done = true;
        }
        return done;
    }

    // Extend arm out until the newPosition reached
    private boolean armRetract(int newPosition) {
        boolean done = false;
        if (extendArmMotor.getCurrentPosition() > newPosition) {
            // retract arm
            extendArmMotor.setPower(-EXTEND_POWER);
        }
        else { //extension is <= newPosition so stop motor
            extendArmMotor.setPower(0);
            done = true;
        }
        return done;
    }

    private void displayDebugInfo() {
        telemetry.addData("State: ", stateMachine);
        telemetry.addData("xPod: ", odo.getEncoderX());
        telemetry.addData("yPod: ", odo.getEncoderY());
        telemetry.addData("Pose X(in): ", odo.getPosition().getX(DistanceUnit.INCH));
        telemetry.addData("Pose Y(in): ", odo.getPosition().getY(DistanceUnit.INCH));
        telemetry.addData("Pose Heading(deg): ", odo.getPosition().getHeading(AngleUnit.DEGREES));
        telemetry.addData("ArmRotCurrPosition", rotateArmMotor.getCurrentPosition());
        telemetry.addData("ArmRotTargetPosition", armTargetRotPos);
        telemetry.addData("ExtensionCurrPosition", extendArmMotor.getCurrentPosition());
        telemetry.addData("ExtensionTargetPosition", armTargetExtPos);
        telemetry.update();
    }

    private void initPinpoint() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(132, 142); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                                 GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
    }
}
