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
    DcMotor motorIntake;
    DcMotor motorWinch;
    DcMotor motorArm;
    GyroSensor gyro;
    ColorSensor color;
    DcMotorController driveController;
    public ElapsedTime msecClock = new ElapsedTime();





    @Override
    public void runOpMode() throws InterruptedException {

        motorRight = hardwareMap.dcMotor.get("rightMotor");
        motorLeft = hardwareMap.dcMotor.get("leftMotor");
        //motorIntake = hardwareMap.dcMotor.get("intake");
        //motorWinch = hardwareMap.dcMotor.get("winch");
        //motorArm = hardwareMap.dcMotor.get("arm");
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        driveController = hardwareMap.dcMotorController.get("MC0");
        //gyro = hardwareMap.gyroSensor.get("gyro");
        //color = hardwareMap.colorSensor.get("color");

        driveController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
        while(driveController.getMotorControllerDeviceMode() != DcMotorController.DeviceMode.READ_ONLY) {
            waitOneFullHardwareCycle();
        }
        wait1MSec(250);

        initial = motorLeft.getCurrentPosition();

        driveController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);
        while(driveController.getMotorControllerDeviceMode() != DcMotorController.DeviceMode.WRITE_ONLY) {
            waitOneFullHardwareCycle();
        }
        wait1MSec(250);

        //driveController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);

        waitForStart();
        /*telemetry.addData("", "Deciding...");
        wait1MSec(3000);
        telemetry.clearData();
        telemetry.addData("", "The color is" + decideColor());
        */
        driveWithEncoders(2, .5);
        wait1MSec(1000);
        driveWithEncoders(2, -.5);




    }

    public String decideColor() {
        String out = "";

        if(color.red() < color.blue()) {
            out = "blue";
        } else {
            out = "red";
        }

        return out;

    }


    public void setDriveSpeed(double left, double right) {
        motorLeft.setPower(left);
        motorRight.setPower(right);
    }

    public void waitForEncoders() throws InterruptedException{

        driveController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
        while(driveController.getMotorControllerDeviceMode() != DcMotorController.DeviceMode.READ_ONLY) {
            waitOneFullHardwareCycle();
        }

        while(Math.abs(motorLeft.getCurrentPosition() - target) > 5) {
            telemetry.addData("target", target);
            telemetry.addData("enc", Math.abs(motorLeft.getCurrentPosition() - target));
            telemetry.addData("initial", initial);
            initial = motorLeft.getCurrentPosition();
        }



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



    public void driveWithEncoders(double tiles, double power) throws InterruptedException{
        int multiplier = (int)(power/Math.abs(power));
        int targetPos;
        int ticks = (int)(((tiles * INCHES_PER_TILE)/WHEEL_CIRCUMFERENCE) * ENCODER_TICKS_PER_REVOLUTION);
        targetPos = (ticks * multiplier);



        setTarget(targetPos);
        setDriveSpeed(power, power);
        wait1MSec(100);
        waitForEncoders();
    }

    public void turnWithGyro(int degreesToTurn) throws InterruptedException{
        double MOTOR_POWER = 0.5;
        double degreesTurned = 0;
        double initial = gyro.getRotation();

        while(Math.abs(degreesTurned) < Math.abs(degreesToTurn)) {
            sleep(20);

            double rotSpeed = (gyro.getRotation() - initial) * .02;
            degreesTurned += rotSpeed;

            motorLeft.setPower(MOTOR_POWER);
            motorRight.setPower(-MOTOR_POWER);
        }
        setDriveSpeed(0, 0);
    }

}
