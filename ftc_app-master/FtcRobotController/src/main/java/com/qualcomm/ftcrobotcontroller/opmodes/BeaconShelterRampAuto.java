package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;


public class BeaconShelterRampAuto extends LinearOpMode {

    int ENCODER_TICKS_PER_REVOLUTION = 1220;
    int INCHES_PER_TILE = 24;
    double WHEEL_DIAMETER = 4;
    double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    int target = 0;
    int initial = 0;


    DcMotor motorLeft;
    DcMotor motorRight;
    GyroSensor gyro;
    DcMotorController driveController;
    public ElapsedTime msecClock = new ElapsedTime();





    @Override
    public void runOpMode() throws InterruptedException {

        motorLeft = hardwareMap.dcMotor.get("leftMotor");
        motorRight = hardwareMap.dcMotor.get("rightMotor");
        driveController = hardwareMap.dcMotorController.get("MC0");
        gyro = hardwareMap.gyroSensor.get("gyro");




        //motorLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        driveController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
        while(driveController.getMotorControllerDeviceMode() != DcMotorController.DeviceMode.READ_ONLY) {
            waitOneFullHardwareCycle();
        }

        initial = motorLeft.getCurrentPosition();

        driveController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);
        while(driveController.getMotorControllerDeviceMode() != DcMotorController.DeviceMode.WRITE_ONLY) {
            waitOneFullHardwareCycle();
        }


        //driveController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);

        waitForStart();

        driveWithEncoders(5000, .5);
        wait1MSec(1000);
        driveWithEncoders(5000, .5);
        wait1MSec(1000);
        driveWithEncoders(5000, .5);

        //setTarget(5000);
        //setDriveSpeed(.5, .5);
        //waitForEnocders();







/*
        driveWithEncoders(2);

        sleep(1000);

        driveWithEncoders(2);

        sleep(1000);

        driveWithEncoders(2);

        sleep(1000);

        driveWithEncoders(2);

        sleep(1000);

        driveWithEncoders(2);
*/




    }


    public void setDriveSpeed(double left, double right) {
        motorLeft.setPower(left);
        motorRight.setPower(right);
    }

    public void waitForEncodersForward() throws InterruptedException{

        driveController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
        while(driveController.getMotorControllerDeviceMode() != DcMotorController.DeviceMode.READ_ONLY) {
            waitOneFullHardwareCycle();
        }

        while(Math.abs(motorLeft.getCurrentPosition()) - target < 5) {
            telemetry.addData("enc", Math.abs(motorLeft.getCurrentPosition()) - target);
        }

        initial = motorLeft.getCurrentPosition();

        driveController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);
        while(driveController.getMotorControllerDeviceMode() != DcMotorController.DeviceMode.WRITE_ONLY) {
            waitOneFullHardwareCycle();
        }
        setDriveSpeed(0, 0);

    }

    public void waitForEncodersBackward() throws InterruptedException{

        driveController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
        while(driveController.getMotorControllerDeviceMode() != DcMotorController.DeviceMode.READ_ONLY) {
            waitOneFullHardwareCycle();
        }

        while(Math.abs(motorLeft.getCurrentPosition()) - target > -5) {
            telemetry.addData("enc", Math.abs(motorLeft.getCurrentPosition()) - target);
        }

        initial = motorLeft.getCurrentPosition();

        driveController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);
        while(driveController.getMotorControllerDeviceMode() != DcMotorController.DeviceMode.WRITE_ONLY) {
            waitOneFullHardwareCycle();
        }
        setDriveSpeed(0, 0);

    }

    public void setTarget(int targetVal) {
        target = targetVal + initial;
    }

    public void wait1MSec(int msec) {
        msecClock.reset();
        while((msecClock.time() * 1000) < (msec + 1)) {

        }

    }

    public void driveWithEncoders(int target, double power) throws InterruptedException{
        int multiplier = (int)(power/Math.abs(power));

        target = (int)(target * multiplier);

        setTarget(target);
        setDriveSpeed(power, power);
        wait1MSec(250);
        if(multiplier > 0) {
            waitForEncodersForward();
        }
        else {
            waitForEncodersBackward();
        }

    }
}
