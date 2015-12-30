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
        TURN_TO_BEACON,
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
    double WHEEL_DIAMETER = 4;
    double WHEEL_CIRCUMFERENCE = 3.14159265359 * WHEEL_DIAMETER;
    double GEAR_RATIO = 11/8;
    int ENCODER_TICKS_PER_INCH = (int)((ENCODER_TICKS_PER_REVOLUTION/WHEEL_CIRCUMFERENCE) * GEAR_RATIO);
    int previousLeftEncoder = 0;
    int currentLeftEncoder = 0;
    int previousRightEncoder = 0;
    int currentRightEncoder = 0;

    private boolean firstLoop = true;
    private int degreesTurned = 0;
    private int degreesTurnedThisLoop = 0;
    private int gyroCalibrate = 0;
    private rotationSwag currentTurn;

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

    DigitalChannel limitWinch;
    DigitalChannel limitArm;

    private PathSegment beaconPath[] = {
            new PathSegment(60, 60, 0.5)
    };

    private rotationSwag beaconTurn =  new rotationSwag(-135, 0.5);

    private boolean rightReady = false;
    private boolean leftReady = false;

    @Override
    public void init() {
        motorRight = hardwareMap.dcMotor.get("rightMotor");
        motorLeft = hardwareMap.dcMotor.get("leftMotor");
        motorIntake = hardwareMap.dcMotor.get("intake");
        motorWinch = hardwareMap.dcMotor.get("winch");
        motorArm = hardwareMap.dcMotor.get("arm");
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        motorIntake.setDirection(DcMotor.Direction.REVERSE);
        motorArm.setDirection(DcMotor.Direction.REVERSE);
        motorIntake.setDirection(DcMotor.Direction.REVERSE);
        motorArm.setDirection(DcMotor.Direction.REVERSE);
        limitWinch = hardwareMap.digitalChannel.get("winchLimit");
        limitArm = hardwareMap.digitalChannel.get("armLimit");
        gyro = hardwareMap.gyroSensor.get("gyro");
        driveController = hardwareMap.dcMotorController.get("MC0");
    }

    @Override
    public void init_loop() {
        motorRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        gyroCalibrate = (int)gyro.getRotation();
        telemetry.addData("Mode", driveController.getMotorControllerDeviceMode());
    }

    @Override
    public void start() {
        runTime.reset();
        //newState(STATES.DRIVE_TO_BEACON);
        //startPath(beaconPath);
        addEncoderTarget(5000, 5000);
    }

    @Override
    public void loop() {
    /*
        switch(currentState) {

            case INIT:

                startPath(beaconPath);
                newState(STATES.DRIVE_TO_BEACON);

                break;

            case DRIVE_TO_BEACON:

                if(pathComplete()) {
                    setDrivePower(0, 0);
                    startTurn(beaconTurn);
                    newState(STATES.TURN_TO_BEACON);
                } else {

                    telemetry.addData("", "State: " + currentState + ", Time: " + stateTime.time() + ", Mode: " + driveController.getMotorControllerDeviceMode() + "seg" + currentSegment + "path" + currentPath);
                }
                telemetry.addData("State", currentState);
                telemetry.addData("Path", currentPath);
                telemetry.addData("Segment", currentSegment);
                telemetry.addData("Time", stateTime.time());
                telemetry.addData("Mode", driveController.getMotorControllerDeviceMode());


                break;
            case TURN_TO_BEACON:
                if(turnComplete()) {
                    setDrivePower(0, 0);
                    newState(STATES.STOP);
                }
        }
    */
        motorLeft.setPower(0.5);
        motorRight.setPower(0.5);
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
            left = (int)(currentPath[currentSegment].mLeft * ENCODER_TICKS_PER_INCH);
            right = (int)(currentPath[currentSegment].mRight * ENCODER_TICKS_PER_INCH);
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

            previousLeftEncoder = currentLeftEncoder;
            previousRightEncoder = currentRightEncoder;

        } else {
            driveController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
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

    private void startTurn(rotationSwag rot) {
        turnClock.reset();
        degreesTurned = 0;
        degreesTurnedThisLoop = 0;
        setDrivePower(rot.speed, -rot.speed);
        currentTurn = rot;
    }

    private boolean turnComplete() {
        degreesTurnedThisLoop = (int)((gyro.getRotation() - gyroCalibrate) * turnClock.time());
        degreesTurned += degreesTurnedThisLoop;

        if(Math.abs(degreesTurned) >= Math.abs(currentTurn.degreesToTurn)) {
            currentTurn = null;
            return true;
        }

        return false;
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
        speed = power * (degreesTT/Math.abs(degreesTT));

    }


}