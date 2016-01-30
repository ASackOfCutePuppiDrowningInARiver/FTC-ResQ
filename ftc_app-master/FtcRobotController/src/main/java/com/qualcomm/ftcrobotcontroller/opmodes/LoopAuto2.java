package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LoopAuto2 extends OpMode {
    LightSensor light;

    //----------------------------------------------------------------------------------------------
    // States for state machine
    //----------------------------------------------------------------------------------------------

    public enum STATES {

        //
        INIT,
        RUN,
        TURN1,//-45 degrees
        FORWARD1, //2 TILES
        MOVE_LESS,
        SLOW,
        TURN2, //45 degrees
        SQUARE_UP,
        BEACON_BACKUP,
        LIGHT,
        //Deploy climbers
        ARM1,
        WINCH1,
        ARM2,
        BACKARM1,
        RETRACTWINCH,
        RETRACTARM,
        LAST_TURN,
        R_FORWARD,
        R_TURN,
        RAMP,
        END,
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
    private int winchEncoderTarget;


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
            new PathSegment(140, .5)
    };
    private PathSegment Bforward[] = {
            new PathSegment(-30, .5)
    };
    private PathSegment Bbackward[] = {
            new PathSegment(30, .3)
    };
    private PathSegment Rforward[] = {
            new PathSegment(34, .5)
    };
    private PathSegment Ramp[] = {
            new PathSegment(30,.3)
    };
    private PathSegment BeaconBackward[] = {
            new PathSegment(7,.5)
    };
//make sure numbers are changed according to battery power


    private PathSegment Reverse[] = {
            new PathSegment(-30, .5)
    };
    private PathSegment SquareUp[] ={
            new PathSegment(-30,.5)
    };
    Servo leftBoxDoor;


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
        leftBoxDoor = hardwareMap.servo.get("lDoor");
        resetDriveEncoders();
    }

    @Override
    public void init_loop() {
        runToPosition();
        gyroCalibrate = (int) gyro.getRotation();
        telemetry.addData("Enc", String.format("L %5d - R %5d ", getLeftPosition(), getRightPosition()));
        resetDriveEncoders();
        light = hardwareMap.lightSensor.get("light");
        light.enableLed(true);
        motorArm.setMode(DcMotorController.RunMode.RESET_ENCODERS);
    }

    @Override
    public void start() {
        runTime.reset();
        setDrivePower(0, 0);
        syncEncoders();
        syncArmEncoder();
        syncWinchEncoder();
        runToPosition();
        telemetry.addData("start", "good");
        motorArm.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        newState(STATES.TURN1);
        turn(45);
        /*
        newState(STATES.DRIVE_TO_BEACON);

        startPath(beaconPath);
    */
    }

    @Override
    public void loop() {

        switch (currentState) {

            case TURN1:
                if (turnComplete()){
                    resetTurn();
                        startPath(Forward1);
                    newState(STATES.SLOW);
                }
            else {calculateTurn();}
                break;
            case SLOW:
                if (pathComplete()) {
                    setDrivePower(0,0);
                    turn(0);
                    newState(STATES.MOVE_LESS);
                }
                break;
            case MOVE_LESS:
                if (turnComplete()) {
                    resetTurn();
                    setDrivePower(-.6,.6);
                    newState(STATES.FORWARD1);
                }
                else {calculateTurn();}

            case FORWARD1:
                if (light.getLightDetectedRaw() < .6){
                    setDrivePower(0,0);
                    newState(STATES.TURN2);
                }
                    break;

            case TURN2:
                if (turnComplete()){
                    resetTurn();
                    startPath(SquareUp);
                    newState(STATES.SQUARE_UP);
                }
                else {
                    calculateTurn();
                }
                break;

            case SQUARE_UP:
                if (pathComplete()){
                    startPath(BeaconBackward);
                    newState(STATES.BEACON_BACKUP);
                }
                break;

            case BEACON_BACKUP:
                if (pathComplete()){
                    positionArm(-1000,.5);
                    newState(STATES.ARM1);
                }
                break;

            case ARM1:
                if (armPositioned()){
                    armMove(0);
                    WinchMove(-.5);
                    newState(STATES.WINCH1);
                }
                break;

            case WINCH1:
                if (stateTime.time() > 1.55){
                    WinchMove(0);
                    positionArm(-5100,.3);
                    newState(STATES.ARM2);
                }
                break;

            case ARM2:
                if (armPositioned() && stateTime.time() > 5){
                    startPath(Bbackward);
                    WinchMove(.5);
                    newState(STATES.RETRACTWINCH);
                }
                break;

            case RETRACTWINCH:
                if (pathComplete() && winchLimited()){
                    setDrivePower(0,0);
                    newState(STATES.LAST_TURN);
                }

                    break;

            case RETRACTARM:
                if (winchLimited()){
                    WinchMove(0);
                    positionArm(0,.5);
                    newState(STATES.LAST_TURN);
                }
                break;
            case LAST_TURN:
                if (armPositioned()){
                    armMove(0);
                    turn(-95);
                    newState(STATES.R_TURN);
                }
                else {
                    calculateTurn();
                }
                break;

            case R_TURN:
                if (pathComplete()){
                    setDrivePower(0,0);
                    turn(-45);
                    newState(STATES.RAMP);
                }
                break;
            case RAMP:
                if (turnComplete()){
                    resetTurn();
                    startPath(Ramp);
                    newState(STATES.END);
                }
                break;
            case END:
                if (pathComplete()){
                    setDrivePower(0,0);
                    newState(STATES.STOP);
                }
                break;
//make the arm move instead of winch
            case STOP:
                useConstantPower();
                setDrivePower(0, 0);
                WinchMove(0);
                armMove(0);
                telemetry.addData("mission", "success");
                break;
                }

        telemetry.addData("state", currentState);
        telemetry.addData("left", motorLeft.getPower());
                telemetry.addData("right", motorRight.getPower());
        telemetry.addData("Time", runTime.time());
        telemetry.addData("arm", motorArm.getCurrentPosition());
        telemetry.addData("light", light.getLightDetected());
    }

    @Override
    public void stop() {
        motorArm.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
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
    private void syncArmEncoder() {
        armEncoderTarget = motorArm.getCurrentPosition();
    }
    private void syncWinchEncoder() {
        winchEncoderTarget = motorWinch.getCurrentPosition();
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
    private  void armMove(double power) {
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
        //syncArmEncoder();
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

   private void setwinchEncoderTarget(int winchEncoder) {
        motorWinch.setTargetPosition(winchEncoderTarget = winchEncoder);
    }
    private void positionWinch(int position, double speed) {
        motorWinch.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        setwinchEncoderTarget(position);
        motorWinch.setPower(speed);
    }
    private void addWinchEncoderTarget(int winchEncoder) {
        motorWinch.setTargetPosition(winchEncoderTarget += winchEncoder);
    }
    void setWinchEncoderTarget(int winchEncoder) {
        motorWinch.setTargetPosition(winchEncoderTarget = winchEncoder);
    }
    private boolean winchPositioned() {
        //return !motorWinch.isBusy();
        return (Math.abs(getwinchPosition() - winchEncoderTarget) < 10);
    }
    int getwinchPosition() {
        return motorWinch.getCurrentPosition();
    }
    private boolean winchLimited() {
        return !limitWinch.getState();
    }




}


