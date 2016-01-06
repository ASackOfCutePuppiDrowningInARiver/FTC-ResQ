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
        resetDriveEncoders();
    }

    @Override
    public void init_loop() {
        motorRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        gyroCalibrate = (int)gyro.getRotation();
        telemetry.addData("Enc", String.format("L %5d - R %5d ", getLeftPosition(), getRightPosition()));
        resetDriveEncoders();
    }

    @Override
    public void start() {
        runTime.reset();
        setDrivePower(0, 0);
        runToPosition();
        newState(STATES.INIT);
        startPath(beaconPath);
    }

    @Override
    public void loop() {

        switch(currentState) {

            case INIT:
                if(encodersAtZero()) {
                    startPath(beaconPath);
                    newState(STATES.DRIVE_TO_BEACON);
                } else {
                    telemetry.addData("Enc", String.format("L %5d - R %5d ", getLeftPosition(), getRightPosition() ));
                }
                break;

            case DRIVE_TO_BEACON:

                if(pathComplete()) {
                    setDrivePower(0, 0);
                    startTurn(beaconTurn);
                    newState(STATES.TURN_TO_BEACON);
                } else {

                    telemetry.addData("", "State: " + currentState + ", Time: " + stateTime.time() + "seg" + currentSegment + "path" + currentPath);
                }
                break;

            case TURN_TO_BEACON:
                if(turnComplete()) {
                    setDrivePower(0, 0);
                    newState(STATES.STOP);
                }
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

    void setEncoderTarget(int leftEncoder, int rightEncoder) {
        motorLeft.setTargetPosition(leftEncoderTarget = leftEncoder);
        motorRight.setTargetPosition(rightEncoderTarget = rightEncoder);
    }

    private void addEncoderTarget(int leftEncoder, int rightEncoder) {
        motorLeft.setTargetPosition(leftEncoderTarget += leftEncoder);
        motorRight.setTargetPosition(rightEncoderTarget += rightEncoder);
    }

    void syncEncoders()
    {
        leftEncoderTarget = motorLeft.getCurrentPosition();
        rightEncoderTarget = motorRight.getCurrentPosition();
    }

    boolean encodersAtZero()
    {
        return ((Math.abs(getLeftPosition()) < 5) && (Math.abs(getRightPosition()) < 5));
    }


    public void runToPosition()
    {
        setDriveMode(DcMotorController.RunMode.RUN_TO_POSITION);
    }

    public void useConstantSpeed()
    {
        setDriveMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
    }

    public void useConstantPower()
    {
        setDriveMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
    }

    public void resetDriveEncoders()
    {
        setEncoderTarget(0, 0);
        setDriveMode(DcMotorController.RunMode.RESET_ENCODERS);
    }

    public void setDriveMode(DcMotorController.RunMode mode)
    {
        // Ensure the motors are in the correct mode.
        if (motorLeft.getMode() != mode)
            motorLeft.setMode(mode);

        if (motorRight.getMode() != mode)
            motorRight.setMode(mode);
    }

    int getLeftPosition()
    {
        return motorLeft.getCurrentPosition();
    }

    int getRightPosition()
    {
        return motorRight.getCurrentPosition();
    }

    private void startPath(PathSegment[] path) {
        currentPath = path;
        currentSegment = 0;
        syncEncoders();
        runToPosition();
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
        }
    }

    private boolean moveComplete() {
        //  return (!mLeftMotor.isBusy() && !mRightMotor.isBusy());
        return ((Math.abs(getLeftPosition() - leftEncoderTarget) < 10) &&
                (Math.abs(getRightPosition() - rightEncoderTarget) < 10));
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