package com.qualcomm.ftcrobotcontroller.opmodes;

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
    double WHEEL_DIAMETER = 4;
    double WHEEL_CIRCUMFERENCE = 3.14159265359 * WHEEL_DIAMETER;
    double GEAR_RATIO = 11/8;
    int ENCODER_TICKS_PER_INCH = (int)((ENCODER_TICKS_PER_REVOLUTION/WHEEL_CIRCUMFERENCE) * GEAR_RATIO);


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
    private int armEncoderTarget;


    DcMotor motorLeft;
    DcMotor motorRight;
    DcMotor motorIntake;
    DcMotor motorWinch;
    DcMotor motorArm;
    GyroSensor gyro;
    ColorSensor color;

    DigitalChannel limitWinch;
    DigitalChannel limitArm;

    private double gyroCalibrate;

    private int armPrepare = 100;
    private int armDeploy = 300;

    private PathSegment beaconPath[] = {
            new PathSegment(60, 0.5), //drive to beacon
            new PathSegment(-135) //turn so that the arm is facing the beacon area
    };


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
        runToPosition();
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
                    positionArm(armPrepare, 0.8);
                    newState(STATES.PREPARE_EXTENSION);
                } else {
                    //add telemetry
                    //if on the way to beacon, run outtake to get debris out the way
                    if(currentSegment == 1) {
                        motorIntake.setPower(-1);
                    } else {
                        motorIntake.setPower(0);
                    }
                }
                break;

            case PREPARE_EXTENSION:

                if(armPositioned()) {
                    motorArm.setPower(0);
                    motorWinch.setPower(1);
                    newState(STATES.EXTEND_ARM);
                } else {
                    //telemetry
                }
                break;

            case EXTEND_ARM:

                if(stateTime.time() > 3) {
                    motorWinch.setPower(0);
                    positionArm(armDeploy, 0.8);
                    newState(STATES.DEPLOY_ARM);
                } else {
                    //telemetry
                }

                break;

            case DEPLOY_ARM:

                break;

            case STOP:
                setDrivePower(0, 0);
                useConstantPower();
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

    void setArmEncoderTarget(int armEncoder) {
        motorArm.setTargetPosition(armEncoderTarget = armEncoder);
    }

    private void addArmEncoderTarget(int armEncoder) {
        motorArm.setTargetPosition(armEncoderTarget += armEncoder);
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

    int getArmPosition() {
        return motorArm.getCurrentPosition();
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
            if(!currentPath[currentSegment].isTurning) {
                left = (int) (currentPath[currentSegment].mLeft * ENCODER_TICKS_PER_INCH);
                right = (int) (currentPath[currentSegment].mRight * ENCODER_TICKS_PER_INCH);
                addEncoderTarget(left, right);
                setDrivePower(currentPath[currentSegment].mSpeed, currentPath[currentSegment].mSpeed);
            } else {
                setDrivePower(currentPath[currentSegment].mSpeed * currentPath[currentSegment].turnDirection, -(currentPath[currentSegment].mSpeed * currentPath[currentSegment].turnDirection));
                turnClock.reset();
            }
            currentSegment++;
        }
    }

    private boolean moveComplete() {
        if(!currentPath[currentSegment].isTurning) {
            //  return (!mLeftMotor.isBusy() && !mRightMotor.isBusy()); use when isBusy is fixed, currently fixed in beta version, this code not yet updated
            return ((Math.abs(getLeftPosition() - leftEncoderTarget) < 10) && (Math.abs(getRightPosition() - rightEncoderTarget) < 10));
        } else {
            currentPath[currentSegment].rotSpeed = (turnClock.time() - currentPath[currentSegment].previousTime) * (gyro.getRotation() - gyroCalibrate);
            currentPath[currentSegment].degreesTurned += currentPath[currentSegment].rotSpeed;
            return (Math.abs(currentPath[currentSegment].degreesTurned) >= Math.abs(currentPath[currentSegment].degreesToTurn));
        }
    }

    private boolean pathComplete() {
        if(moveComplete()) {

            if(currentPath[currentSegment].isTurning) {
                setDrivePower(0, 0);
            }

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

    private void positionArm(int position, double speed) {
        motorArm.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        setArmEncoderTarget(position);
        motorArm.setPower(speed);
    }

    private boolean armPositioned() {
        //return !motorArm.isBusy();
        return (Math.abs(getArmPosition() - armEncoderTarget) < 10);
    }
}

class PathSegment {

    public double mLeft;
    public double mRight;
    public double mSpeed;
    public double degreesToTurn;
    public boolean isTurning;
    public double turnDirection;
    public double degreesTurned = 0;
    public double previousTime = 0;
    public double rotSpeed = 0;

    public PathSegment(double inches, double speed) {
        mLeft = inches;
        mRight = inches;
        mSpeed = speed;
        isTurning = false;
    }

    public PathSegment(double degrees) {
        degreesToTurn = degrees;
        mSpeed = 0.5;
        isTurning = true;
        turnDirection = degreesToTurn/Math.abs(degreesToTurn);
    }
}

