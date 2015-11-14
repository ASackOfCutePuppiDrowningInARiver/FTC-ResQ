package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


public class StateMachineAutoTest extends OpMode{

    int ENCODER_TICKS_PER_REVOLUTION = 1220;
    int INCHES_PER_TILE = 24;
    double WHEEL_DIAMETER = 4;
    double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    int leftEncoderTarget = 0;
    int rightEncoderTarget = 0;

    public ElapsedTime  runtime = new ElapsedTime();   // Time into round.

    public ElapsedTime msecClock = new ElapsedTime();

    private ElapsedTime stateTime = new ElapsedTime();  // Time into current state

    private State       currentState;    // Current State Machine State.



    DcMotor motorLeft;
    DcMotor motorRight;
    GyroSensor gyro;
    DcMotorController driveController;


    private enum State {
        STATE_INITIAL,
        STATE_DRIVE,
        STATE_STOP
    }

    public StateMachineAutoTest() {

    }




    @Override
    public void init() {
        motorLeft = hardwareMap.dcMotor.get("leftMotor");
        motorRight = hardwareMap.dcMotor.get("rightMotor");
        driveController = hardwareMap.dcMotorController.get("MC0");
        gyro = hardwareMap.gyroSensor.get("gyro");
        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        setDrivePower(0, 0);
        driveController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
        //resetDriveEncoders();
    }

    @Override
    public void init_loop() {
        //resetDriveEncoders();

        telemetry.addData("ENC", String.format("L:R %d:%d", getLeftPosition(), getRightPosition()));
    }

    @Override
    public void start()
    {
        driveController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);
        // Setup Robot devices, set initial state and start game clock
        setDrivePower(0, 0);        // Set target speed to zero
        runtime.reset();           // Zero game clock
        runToPosition();
        newState(State.STATE_INITIAL);
    }

    @Override
    public void loop() {
        //telemetry.addData();

        switch(currentState) {

            case STATE_INITIAL:

                if(true) {
                    setEncoderTarget(5000, 5000);
                    setDrivePower(.5, .5);
                    switchMCMode(DcMotorController.DeviceMode.READ_ONLY);
                    wait1MSec(1000);
                    newState(State.STATE_DRIVE);
                }

                break;

            case STATE_DRIVE:

                if(targetReached()) {
                    setDrivePower(0, 0);
                    newState(State.STATE_STOP);
                }

                break;

            case STATE_STOP:
                break;

        }
    }


    private void setDrivePower(double leftPower, double rightPower) {
        motorLeft.setPower(leftPower);
        motorRight.setPower(rightPower);
    }

    private void setEncoderTarget(int leftEncoder, int rightEncoder)
    {
        motorLeft.setTargetPosition(leftEncoderTarget = leftEncoder);
        motorRight.setTargetPosition(rightEncoderTarget = rightEncoder);
    }



    private void newState(State newState)
    {
        // Reset the state time, and then change to next state.
        stateTime.reset();
        currentState = newState;
    }


    //--------------------------------------------------------------------------
    // addEncoderTarget( LeftEncoder, RightEncoder);
    // Sets relative Encoder Position.  Offset current targets with passed data
    //--------------------------------------------------------------------------
    void addEncoderTarget(int leftEncoder, int rightEncoder)
    {
        motorLeft.setTargetPosition(leftEncoderTarget += leftEncoder);
        motorRight.setTargetPosition(rightEncoderTarget += rightEncoder);
    }

    //--------------------------------------------------------------------------
    // setDrivePower( LeftPower, RightPower);
    //--------------------------------------------------------------------------


    //--------------------------------------------------------------------------
    // setDriveSpeed( LeftSpeed, RightSpeed);
    //--------------------------------------------------------------------------

    // runToPosition ()
    // Set both drive motors to encoder servo mode (requires encoders)
    //--------------------------------------------------------------------------
    public void runToPosition()
    {
        setDriveMode(DcMotorController.RunMode.RUN_TO_POSITION);
    }

    //--------------------------------------------------------------------------
    // useConstantSpeed ()
    // Set both drive motors to constant speed (requires encoders)
    //--------------------------------------------------------------------------
    public void useConstantSpeed()
    {
        setDriveMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
    }

    //--------------------------------------------------------------------------
    // useConstantPower ()
    // Set both drive motors to constant power (encoders NOT required)
    //--------------------------------------------------------------------------
    public void useConstantPower()
    {
        setDriveMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
    }

    //--------------------------------------------------------------------------
    // resetDriveEncoders()
    // Reset both drive motor encoders, and clear current encoder targets.
    //--------------------------------------------------------------------------
    public void resetDriveEncoders()
    {
        setEncoderTarget(0, 0);
        setDriveMode(DcMotorController.RunMode.RESET_ENCODERS);
    }

    //--------------------------------------------------------------------------
    // syncEncoders()
    // Load the current encoder values into the Target Values
    // Essentially synch's the software with the hardware
    //--------------------------------------------------------------------------


    //--------------------------------------------------------------------------
    // setDriveMode ()
    // Set both drive motors to new mode if they need changing.
    //--------------------------------------------------------------------------
    public void setDriveMode(DcMotorController.RunMode mode)
    {
        // Ensure the motors are in the correct mode.
        driveController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);

        if (motorLeft.getMode() != mode || motorRight.getMode() != mode) {

            motorLeft.setMode(mode);
            motorRight.setMode(mode);


        }
        driveController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);

    }

    //--------------------------------------------------------------------------
    // getLeftPosition ()
    // Return Left Encoder count
    //--------------------------------------------------------------------------
    int getLeftPosition()
    {
        return motorLeft.getCurrentPosition();
    }

    //--------------------------------------------------------------------------
    // getRightPosition ()
    // Return Right Encoder count
    //--------------------------------------------------------------------------
    int getRightPosition()
    {
        return motorRight.getCurrentPosition();
    }

    boolean encodersAtZero()
    {
        driveController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);



        int left = Math.abs(getLeftPosition());
        int right= Math.abs(getRightPosition());

        driveController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);

        return (left < 5 && right < 5);


    }

    boolean targetReached() {

        driveController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);


        int left = motorLeft.getCurrentPosition();
        int right = motorRight.getCurrentPosition();

        driveController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);

        return (leftEncoderTarget - Math.abs(left) < 5 && rightEncoderTarget - Math.abs(right) < 5);
    }


    public void wait1MSec(int msec) {
        msecClock.reset();
        while((msecClock.time() * 1000) < (msec + 1)) {

        }

    }

    public void switchMCMode(DcMotorController.DeviceMode mode) {
        msecClock.reset();
        while((msecClock.time() * 1000) < 300) {
            driveController.setMotorControllerDeviceMode(mode);
        }

    }

}
