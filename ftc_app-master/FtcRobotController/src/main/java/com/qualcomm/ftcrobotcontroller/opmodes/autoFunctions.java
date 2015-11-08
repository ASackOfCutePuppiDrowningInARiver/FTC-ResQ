/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class autoFunctions extends LinearOpMode{




	/**
	 * Constructor
	 */

    Class currentClass;

    public autoFunctions() {

	}


    @Override
    public void runOpMode() throws InterruptedException {

    }

    int ENCODER_TICKS_PER_REVOLUTION = 1440;
    double WHEEL_DIAMETER = 4;
    double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;

    DcMotor motorLeft;
    DcMotor motorRight;
    GyroSensor gyro;

    public void motorAndSensorSetup(String leftMotor, String rightMotor, String GyroSensor) {
        motorLeft = hardwareMap.dcMotor.get(leftMotor);
        motorRight = hardwareMap.dcMotor.get(rightMotor);
        gyro = hardwareMap.gyroSensor.get(GyroSensor);
    }

    public void initializerobot() {
        motorLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorLeft.setPower(0);
        motorRight.setPower(0);
    }

    public void resetEncoders() {
        motorLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);
    }






    public void turnWithGyro(int degreesToTurn) throws InterruptedException{
        double degreesTurned = 0;

        double initial = gyro.getRotation();


        double MOTOR_POWER = 0.5 * (degreesToTurn/Math.abs(degreesToTurn));

        while(Math.abs(degreesTurned) < Math.abs(degreesToTurn)) {
            sleep(20);

            double rotSpeed = gyro.getRotation() - initial;
            degreesTurned += rotSpeed * 0.02;

            motorLeft.setPower(MOTOR_POWER);
            motorRight.setPower(-MOTOR_POWER);

            telemetry.addData("degrees", degreesTurned);
        }
        motorLeft.setPower(0);
        motorRight.setPower(0);
    }

    public void driveStraight(double tiles, double power) throws InterruptedException {
        resetEncoders();
        int encoderTicks = (int) ((tiles * 1440) / WHEEL_CIRCUMFERENCE);
        boolean stopMotors = true;

        while (Math.abs(motorLeft.getCurrentPosition()) < encoderTicks) {
            motorLeft.setPower(power);
            motorRight.setPower(power);
        }

        if(stopMotors) {
            motorLeft.setPower(0);
            motorRight.setPower(0);
        }
    }

    public void driveStraight(double tiles, double power, boolean stopMotors) throws InterruptedException {
        resetEncoders();
        int encoderTicks = (int) ((tiles * 1440) / WHEEL_CIRCUMFERENCE);

        while (Math.abs(motorLeft.getCurrentPosition()) < encoderTicks) {
            motorLeft.setPower(power);
            motorRight.setPower(power);
        }

        if(stopMotors) {
            motorLeft.setPower(0);
            motorRight.setPower(0);
        }
    }







}
