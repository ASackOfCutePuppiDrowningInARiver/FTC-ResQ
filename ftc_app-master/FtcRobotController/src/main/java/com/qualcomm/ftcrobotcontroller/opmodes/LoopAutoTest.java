package com.qualcomm.ftcrobotcontroller.opmodes;

import android.graphics.Path;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LoopAutoTest extends OpMode {

    //----------------------------------------------------------------------------------------------
    // States for state machine
    //----------------------------------------------------------------------------------------------

    public enum STATES {
        INIT,
        DRIVE_TO_BEACON,
        PREPARE_EXTENSION,
        EXTEND_ARM,
        DEPLOY_ARM,
        WITHDRAW_ARM,
        RETRACT_ARM,
        RESET_ARM,
        STOP
    }

    //----------------------------------------------------------------------------------------------
    //Constants for converting inches to encoder ticks
    //----------------------------------------------------------------------------------------------

    int ENCODER_TICKS_PER_REVOLUTION = 1120;
    int INCHES_PER_TILE = 24;
    double WHEEL_DIAMETER = 4;
    double WHEEL_CIRCUMFERENCE = 3.14159265359 * WHEEL_DIAMETER;
    double GEAR_RATIO = 11/8;
    double target = 0;
    int initial = 0;
    int initialtemp = 0;
    int previousLeftEncoder = 0;
    int currentLeftEncoder = 0;
    int previousRightEncoder = 0;
    int currentRightEncoder = 0;

    private boolean firstLoop = true;
    private int degreesTurned = 0;
    private int degreesTurnedThisLoop = 0;

    //----------------------------------------------------------------------------------------------
    //
    //----------------------------------------------------------------------------------------------

    private STATES currentState;
    private PathSegment[] currentPath;
    private int currentSegment;

    private ElapsedTime stateTime = new ElapsedTime();
    private ElapsedTime runTime = new ElapsedTime();
    private ElapsedTime turnClock = new ElapsedTime();

    private int leftEncoderTarget;
    private int rightEncoderTarget;


    DcMotor motorLeft;
    DcMotor motorRight;
    DcMotor motorIntake;
    DcMotor motorWinch;
    DcMotor motorArm;
    GyroSensor gyro;
    ColorSensor color;
    DcMotorController driveController;
    DcMotorController encoderController;
    DcMotor leftEncoder;
    DcMotor rightEncoder;
    DcMotor armEncoder;

    DigitalChannel limitWinch;
    DigitalChannel limitArm;

    DcMotor motorLeftTwo;

    private PathSegment beaconPath[] = {
            new PathSegment(60, 60, 0.5)
    };

    @Override
    public void init() {

        motorLeftTwo = hardwareMap.dcMotor.get("l2");


        /*
        motorRight = hardwareMap.dcMotor.get("rightMotor");
        motorLeft = hardwareMap.dcMotor.get("leftMotor");
        motorIntake = hardwareMap.dcMotor.get("intake");
        motorWinch = hardwareMap.dcMotor.get("winch");
        motorArm = hardwareMap.dcMotor.get("arm");
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        //motorLeftTwo = hardwareMap.dcMotor.get("l2");
        //motorLeftTwo.setDirection(DcMotor.Direction.REVERSE);
        motorIntake.setDirection(DcMotor.Direction.REVERSE);
        motorArm.setDirection(DcMotor.Direction.REVERSE);
        limitWinch = hardwareMap.digitalChannel.get("winchLimit");
        limitArm = hardwareMap.digitalChannel.get("armLimit");
        */
    }

    @Override
    public void init_loop() {
        motorRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void start() {
        runTime.reset();
        newState(STATES.INIT);
    }

    @Override
    public void loop() {






        switch(currentState) {

            case INIT:

                startPath(beaconPath);
                newState(STATES.DRIVE_TO_BEACON);

                break;

            case DRIVE_TO_BEACON:

                if(pathComplete()) {
                    setDrivePower(0, 0);
                }
                break;
        }
    }

    @Override
    public void stop() {

    }

    public void newState(STATES newState) {
        //reset the state time and set the new state as the current state
        stateTime.reset();
        currentState = newState;
    }

    public void setDrivePower(double powerL, double powerR) {
        motorLeft.setPower(powerL);
        motorRight.setPower(powerR);
    }

    private void addEncoderTarget(int leftEncoder, int rightEncoder) {
        motorLeft.setTargetPosition(leftEncoderTarget += leftEncoder);
        motorRight.setTargetPosition(rightEncoderTarget += rightEncoder);
    }

    //wants to go, check if encoders are at position, and then switch to write mode.
/*
    public void driveWithEncoders (int target) {
        boolean there = false;
        addEncoderTarget(target, target);
        motorLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        encoderController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
        if (target < encoderController.getMotorCurrentPosition(2)) {
            there = true;
            encoderController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);
        } else {
            encoderController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
        }
        if (there) {
            setDrivePower(0, 0);
        }
    }

        //break down
   public void run () {
       motorLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
       motorRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
   }
    public void read () {
        encoderController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
    }

    public void finish () {
        boolean there = false;
        boolean left = false;
        boolean right = false;
        if (target < motorLeft.getCurrentPosition()) {
            left = true;
        }
        if (target < motorRight.getCurrentPosition()) {
            right = true;
        }
        if (left && right) {
            there = true;
        }

        if (there) {
            setDrivePower(0, 0);
        }
    }
    public void driveWithEncoders2 (int target) {
        addEncoderTarget(target,target);
        run();
        read();
        finish();
    }
*/
    public void dumpClimbers () {

    }

    private void startPath(PathSegment[] path) {
        currentPath = path;
        currentSegment = 0;
        //syncEncoders
        //runtoposition
        startSegment();
    }

    private void startSegment() {
        int left;
        int right;

        if(currentPath != null) {
            left = (int)(currentPath[currentSegment].mLeft);
            right = (int)(currentPath[currentSegment].mRight);
            addEncoderTarget(left, right);
            setDrivePower(currentPath[currentSegment].mSpeed, currentPath[currentSegment].mSpeed);
            currentSegment++;
            driveController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);

        }
    }

    private boolean moveComplete() {

        boolean leftIsDone = false;
        boolean rightIsDone = false;

        if(driveController.getMotorControllerDeviceMode() == DcMotorController.DeviceMode.READ_ONLY) {
            currentLeftEncoder = motorLeft.getCurrentPosition();
            currentRightEncoder = motorRight.getCurrentPosition();

            if(Math.abs(currentLeftEncoder - previousLeftEncoder) < 3) {
                leftIsDone = true;
            }

            if(Math.abs(currentRightEncoder - previousRightEncoder) < 3) {
                rightIsDone = true;
            }
        }

        if((leftIsDone && rightIsDone) && driveController.getMotorControllerDeviceMode() != DcMotorController.DeviceMode.WRITE_ONLY) {
            driveController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);
        }

        return ((leftIsDone && rightIsDone) &&  driveController.getMotorControllerDeviceMode() == DcMotorController.DeviceMode.WRITE_ONLY);
    }

    private boolean pathComplete() {
        if(moveComplete()) {

            if(currentSegment < currentPath.length) {
                startSegment();
            } else {
                currentPath = null;
                currentSegment = 0;
                setDrivePower(0, 0);
                return true;
            }
        }

        return false;
    }

    private void turn(rotationSwag rot) {

        if(firstLoop) {
            firstLoop = false;
            turnClock.reset();
            degreesTurned = 0;
            degreesTurnedThisLoop = 0;
        }
        setDrivePower(rot.speed, -rot.speed);
        degreesTurnedThisLoop = (int)(gyro.getRotation() * turnClock.time());
        degreesTurned += degreesTurnedThisLoop;
    }

    private boolean turnComplete(rotationSwag rot) {
        return (degreesTurned >= rot.degreesToTurn);
    }


}

class PathSegment {

    public double mLeft;
    public double mRight;
    public double mSpeed;

    public PathSegment(double left, double right, double speed) {
        mLeft = left;
        mRight = right;
        mSpeed = speed;
    }

}

class rotationSwag{
    public int degreesToTurn;
    public double speed;

    public rotationSwag(int degreesTT, double power) {
        degreesToTurn = degreesTT;
        speed = power;
    }
}