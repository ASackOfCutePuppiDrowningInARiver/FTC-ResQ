package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;


public class BeaconShelterRampAuto extends LinearOpMode {

    int ENCODER_TICKS_PER_REVOLUTION = 1220;
    int INCHES_PER_TILE = 24;
    double WHEEL_DIAMETER = 4;
    double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;


    DcMotor motorLeft;
    DcMotor motorRight;
    GyroSensor gyro;
    DcMotorController driveController;



    public void driveWithEncoders(double tiles) throws InterruptedException {
        int encoderTicks = (int) (((tiles * INCHES_PER_TILE) / WHEEL_CIRCUMFERENCE) * ENCODER_TICKS_PER_REVOLUTION);
        int lastFinal = 0;
        int leftEncoderSwag = 0;


        driveController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);

        while(driveController.getMotorControllerDeviceMode() != DcMotorController.DeviceMode.WRITE_ONLY) {
            waitOneFullHardwareCycle();
        }


        motorLeft.setPower(.3);
        motorRight.setPower(.3);

        driveController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
        waitOneFullHardwareCycle();




        if(motorLeft.getPower() == 0 || motorRight.getPower() == 0) {
            driveController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);

            waitOneFullHardwareCycle();


            motorLeft.setPower(.3);
            motorRight.setPower(.3);

            //telemetry.addData("MC", driveController.getMotorControllerDeviceMode());

            driveController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
            waitOneFullHardwareCycle();
        }

        // motorLeft.setTargetPosition(1220);


        waitOneFullHardwareCycle();

        boolean checkFinal = true;

        while (Math.abs(leftEncoderSwag) < encoderTicks) {
            waitOneFullHardwareCycle();

            if(checkFinal) {
                lastFinal = motorLeft.getCurrentPosition();
                checkFinal = !checkFinal;
            }


            leftEncoderSwag = motorLeft.getCurrentPosition() - lastFinal;
            telemetry.addData("start", lastFinal);
            telemetry.addData("current", motorLeft.getCurrentPosition());
            //telemetry.addData("mode", MC.getMotorControllerDeviceMode());
            lastFinal = motorLeft.getCurrentPosition();
        }
        //telemetry.clearData();

        driveController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);
        while (driveController.getMotorControllerDeviceMode() == DcMotorController.DeviceMode.READ_ONLY) {
            waitOneFullHardwareCycle();
        }

        motorLeft.setPower(0);
        motorRight.setPower(0);


    }


    @Override
    public void runOpMode() throws InterruptedException {

        motorLeft = hardwareMap.dcMotor.get("leftMotor");
        motorRight = hardwareMap.dcMotor.get("rightMotor");
        driveController = hardwareMap.dcMotorController.get("MC0");
        gyro = hardwareMap.gyroSensor.get("gyro");

        //motorLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        //driveController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);

        waitForStart();


        driveWithEncoders(2);

        sleep(1000);

        driveWithEncoders(2);

        sleep(1000);

        driveWithEncoders(2);

        sleep(1000);

        driveWithEncoders(2);

        sleep(1000);

        driveWithEncoders(2);





    }
}
