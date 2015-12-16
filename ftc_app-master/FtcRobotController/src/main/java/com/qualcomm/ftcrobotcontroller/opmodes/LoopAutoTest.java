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
    int INCHES_PER_TILE = 24;
    double WHEEL_DIAMETER = 4;
    double WHEEL_CIRCUMFERENCE = 3.14159265359 * WHEEL_DIAMETER;
    double GEAR_RATIO = 11/8;
    double target = 0;
    int initial = 0;
    int initialtemp = 0;

    //----------------------------------------------------------------------------------------------
    //
    //----------------------------------------------------------------------------------------------

    private STATES currentState;

    private ElapsedTime stateTime = new ElapsedTime();
    private ElapsedTime runTime = new ElapsedTime();
    public ElapsedTime msecClock = new ElapsedTime();
    public ElapsedTime auxClock = new ElapsedTime();



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
    DcMotor armEncoder;

    DigitalChannel limitWinch;
    DigitalChannel limitArm;

    @Override
    public void init() {
        motorRight = hardwareMap.dcMotor.get("rightMotor");
        motorLeft = hardwareMap.dcMotor.get("leftMotor");
        motorIntake = hardwareMap.dcMotor.get("intake");
        motorWinch = hardwareMap.dcMotor.get("winch");
        motorArm = hardwareMap.dcMotor.get("arm");
        //motorLeftTwo = hardwareMap.dcMotor.get("l2");
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        //motorLeftTwo.setDirection(DcMotor.Direction.REVERSE);
        motorIntake.setDirection(DcMotor.Direction.REVERSE);
        motorArm.setDirection(DcMotor.Direction.REVERSE);
        limitWinch = hardwareMap.digitalChannel.get("winchLimit");
        limitArm = hardwareMap.digitalChannel.get("armLimit");
    }

    @Override
    public void init_loop() {

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

                

                break;

            case DRIVE_TO_BEACON:


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

    public void setDrivePower(double power) {
        motorLeft.setPower(power);
        motorRight.setPower(power);
    }


}
