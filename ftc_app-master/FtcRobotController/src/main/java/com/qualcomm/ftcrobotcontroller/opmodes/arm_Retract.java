package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class arm_Retract extends OpMode {

    //----------------------------------------------------------------------------------------------
    // States for state machine
    //----------------------------------------------------------------------------------------------

    public enum STATES {

        //
        INIT,
        RUN,
        TURN1,//-45 degrees
        FORWARD1, //2 TILES
        TURN2, //45 degrees
        BEACON_FORWARD,
        //Deploy climbers
        WINCH,
        ARM,
        EXTEND,


        REVERSE,
        STOP
        /*
        INIT,
        DRIVE_TO_BEACON,
        TURN_TO_BEACON,
        APPROACH_BEACON,
        STOP
        */
    }

    //----------------------------------------------------------------------------------------------
    //Constants for converting inches to encoder ticks
    //----------------------------------------------------------------------------------------------

    int ENCODER_TICKS_PER_REVOLUTION = 1120;
    double WHEEL_DIAMETER = 4;
    double WHEEL_CIRCUMFERENCE = 3.14159265359 * WHEEL_DIAMETER;
    double GEAR_RATIO = 11 / 8;
    int ENCODER_TICKS_PER_INCH = (int) ((ENCODER_TICKS_PER_REVOLUTION / WHEEL_CIRCUMFERENCE) * GEAR_RATIO);


    //----------------------------------------------------------------------------------------------
    //
    //----------------------------------------------------------------------------------------------

    private STATES currentState;
    private PathSegment[] currentPath;
    private int currentSegment;

    private ElapsedTime stateTime = new ElapsedTime();
    private ElapsedTime runTime = new ElapsedTime();
    private ElapsedTime turnClock = new ElapsedTime();
    private ElapsedTime winchClock = new ElapsedTime();

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

    private double gyroCalibrate = 0;
    private double degreesToTurn = 0;
    private double degreesTurned = 0;
    private double turnDirection = 0;
    private double TURN_POWER = 0;
    private boolean WinchFin = false;

    private int armPrepare = 100;
    private int armDeploy = 300;

    //temperary telemetry
    boolean good = false;
    boolean spin1 = false;

    private PathSegment beaconPath[] = {
            new PathSegment(147, 0.5), //drive to beacon
            //new PathSegment(20, -0.5)
    };

    private PathSegment approach[] = {
            new PathSegment(-15, 0.5)
    };

    private PathSegment Run[] = {
            new PathSegment(38, .5)
    };
    //forward one, try 1 inch further
    private PathSegment Forward1[] = {
            new PathSegment(95, .5)
    };
    private PathSegment Bforward[] = {
            new PathSegment(-38, .5)
    };

//make sure numbers are changed acording to battery power


    private PathSegment Reverse[] = {
            new PathSegment(-30, .5)
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
        gyroCalibrate = (int) gyro.getRotation();
        telemetry.addData("Enc", String.format("L %5d - R %5d ", getLeftPosition(), getRightPosition()));
        resetDriveEncoders();
    }

    @Override
    public void start() {
        runTime.reset();
        setDrivePower(0, 0);
        runToPosition();
        telemetry.addData("start", "good");
        newState(STATES.WINCH);
        /*
        newState(STATES.DRIVE_TO_BEACON);

        startPath(beaconPath);
    */
    }

    @Override
    public void loop() {

        switch (currentState) {

            case WINCH:
                if (stateTime.time() > 2) {
                    WinchMove(.5);
                    //if (stateTime < (seconds to move) move, else, set power zero
                    newState(STATES.ARM);
                    telemetry.addData("run", "good");
                }
                break;
            case ARM:
                if (winchLimited()) {
                    setDrivePower(0, 0);
                    motorArm.setPower(0);
                    motorWinch.setPower(0);
                    newState(STATES.STOP);
                    telemetry.addData("arm", "IN");
                }

                break;

            case STOP:
                useConstantPower();
                setDrivePower(0, 0);
                motorArm.setPower(0);
                motorWinch.setPower(0);
                motorIntake.setPower(0);
                telemetry.addData("mission", "success");
                break;
        }

        telemetry.addData("state", currentState);
        telemetry.addData("arm", limitArm.getState());
    }


/*
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
                    newState(STATES.TURN_TO_BEACON);
                    turn(145);
                    motorIntake.setPower(0);
                } else {
                    if(currentSegment == 1) {
                        motorIntake.setPower(-1);
                    } else {
                        motorIntake.setPower(0);
                    }
                }
                break;

            case TURN_TO_BEACON:

                if(turnComplete()) {
                    setDrivePower(0, 0);
                    newState(STATES.APPROACH_BEACON);
                    startPath(approach);
                } else {
                    calculateTurn();
                }

                break;

            case APPROACH_BEACON:

                if(pathComplete()) {
                    setDrivePower(0, 0);
                    newState(STATES.STOP);
                } else {

                }
            break;


            case STOP:
                setDrivePower(0, 0);
                useConstantPower();
                break;
        }

        telemetry.addData("state", currentState);
    */


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

    void syncEncoders() {
        leftEncoderTarget = motorLeft.getCurrentPosition();
        rightEncoderTarget = motorRight.getCurrentPosition();
    }

    boolean encodersAtZero() {
        return ((Math.abs(getLeftPosition()) < 5) && (Math.abs(getRightPosition()) < 5));
    }

    public void runToPosition() {
        setDriveMode(DcMotorController.RunMode.RUN_TO_POSITION);
    }

    public void useConstantSpeed() {
        setDriveMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
    }

    public void useConstantPower() {
        setDriveMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
    }

    public void resetDriveEncoders() {
        setEncoderTarget(0, 0);
        setDriveMode(DcMotorController.RunMode.RESET_ENCODERS);
    }

    public void setDriveMode(DcMotorController.RunMode mode) {
        // Ensure the motors are in the correct mode.
        if (motorLeft.getMode() != mode)
            motorLeft.setMode(mode);

        if (motorRight.getMode() != mode)
            motorRight.setMode(mode);
    }

    int getLeftPosition() {
        return motorLeft.getCurrentPosition();
    }

    int getRightPosition() {
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

        if (currentPath != null) {
            left = (int) (currentPath[currentSegment].mLeft * ENCODER_TICKS_PER_INCH);
            right = (int) (currentPath[currentSegment].mRight * ENCODER_TICKS_PER_INCH);
            addEncoderTarget(left, right);
            setDrivePower(currentPath[currentSegment].mSpeed, currentPath[currentSegment].mSpeed);
            currentSegment++;
        }
    }

    private boolean moveComplete() {

        //  return (!mLeftMotor.isBusy() && !mRightMotor.isBusy()); use when isBusy is fixed, currently fixed in beta version, this code not yet updated
        return ((Math.abs(getLeftPosition() - leftEncoderTarget) < 10) && (Math.abs(getRightPosition() - rightEncoderTarget) < 10));

    }

    private boolean pathComplete() {
        if (moveComplete()) {

            if (currentSegment < currentPath.length) {
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

    private void WinchMove(double power) {
        motorWinch.setPower(power);
    }

    private void armMove(double power) {
        motorArm.setPower(power);
    }

    private boolean WinchDone() {
        if (WinchFin) {
            return true;
        }
        else return false;
    }
    private  void ResetWinchFin() {
        WinchFin = false;
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

    private void spin (double degrees) {
        useConstantPower();
        degreesToTurn = degrees;
        if (degreesToTurn < 0) {

        while (Math.abs(degreesToTurn) > degreesTurned) {
            left();
        }
    }
        else {
            while (Math.abs(degreesToTurn) < degreesTurned )
            right();
        }
        spin1 = true;
    }

    private void turn(double degrees) {
        useConstantPower();
        degreesToTurn = degrees;
        //double MOTOR_POWER = ;
        turnClock.reset();

        if (degrees < 0) {
            left();
        } else {
            right();
        }
        spin1 = true;
    }

    private void left() {
        setDrivePower(-0.8, 0.8);
    }

    private void right() {
        setDrivePower(0.8, -0.8);
    }

    private boolean turnComplete() {
        return (Math.abs(degreesTurned) > Math.abs(degreesToTurn));
    }

    private void calculateTurn() {
        degreesTurned = (gyro.getRotation() - gyroCalibrate) * turnClock.time();
    }

    private void resetTurn() {
        degreesTurned = 0;
        degreesToTurn = 0;
        TURN_POWER = 0;
    }
    private boolean armLimited() {
        return !limitArm.getState();
    }
    private boolean winchLimited() {
        return !limitWinch.getState();
    }

}



