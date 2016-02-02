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

public class LoopAuto3 extends OpMode {
    LightSensor light;

    //----------------------------------------------------------------------------------------------
    // States for state machine
    //----------------------------------------------------------------------------------------------

    public enum STATES {

        TURN1,
        FORWORD1,
        TURN2,
        FORWORDLIGHT,
        BTURN,
        STOP,
        SQUAREUP,
        BBACKWARD,
        ARM1,
        WINCH,
        ARM2,





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


    private PathSegment Forward1[] ={
            new PathSegment(120, .5)
    };
    private PathSegment ForwardLight[] ={
            new PathSegment(35, .5)
    };
    private PathSegment SquareUp[] ={
            new PathSegment(-30, .5)
    };
    private PathSegment Bback[] ={
            new PathSegment(10, .5)
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
        turn(50);
        newState(STATES.TURN1);

        /*
        newState(STATES.DRIVE_TO_BEACON);

        startPath(beaconPath);
    */
    }

    @Override
    public void loop() {

        switch (currentState) {





//make the arm move instead of winch
            case TURN1:
                if (turnComplete()) {
                    resetTurn();
                    startPath(Forward1);
                    newState(STATES.FORWORD1);
                }
                else {calculateTurn();}

                break;
            case FORWORD1:
                if (pathComplete()){
                    turn(-50);
                    newState(STATES.TURN2);
                }

                break;
            case TURN2:
                if (turnComplete()) {
                    resetTurn();
                    setDrivePower(.3, .3);
                    newState(STATES.FORWORDLIGHT);
                }
                else {calculateTurn();}

                break;
            case FORWORDLIGHT:
                if (light.getLightDetectedRaw() < 20) {
                    setDrivePower(0, 0);
                    turn(-90);
                    newState(STATES.BTURN);
                }

                break;
            case BTURN:
                if(turnComplete()){
                    resetTurn();
                    startPath(SquareUp);
                    newState(STATES.SQUAREUP);
                }
                else {calculateTurn();}

                break;
            case SQUAREUP:
                if (pathComplete()){
                    setDrivePower(0,0);
                    startPath(Bback);
                    newState(STATES.ARM1);
                }
                break;
            case ARM1:
                if (pathComplete()) {
                    setDrivePower(0,0);
                    positionArm(-1000, .5);
                    newState(STATES.WINCH);
                }
                break;
            case WINCH:
                if (armPositioned()) {
                    motorArm.setPower(.5);
                    newState(STATES.ARM2);
                }
                break;
            case ARM2:
                if (stateTime.time () > 1.55 ) {
                    WinchMove(0);
                    positionArm(-5100, .3);
                    newState(STATES.BBACKWARD);
                }
                break;
            case BBACKWARD:
                if (armPositioned()) {
                    setDrivePower(0,0);
                    newState(STATES.STOP);
                }
                break;
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


