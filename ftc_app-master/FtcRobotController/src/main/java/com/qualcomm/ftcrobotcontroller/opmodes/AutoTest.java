/* Copyright (c) 2015 Qualcomm Technologies Inc

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
import com.qualcomm.robotcore.hardware.IrSeekerSensor;

/**
 * A simple example of a linear op mode that will approach an IR beacon
 */
public class AutoTest extends LinearOpMode {



    DcMotor motorRight;
    DcMotor motorLeft;
    GyroSensor gyro;

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

    public void straight(double seconds, double power) {
        double startTime = time;
        while (time - startTime < seconds) {
            motorLeft.setPower(power);
            motorRight.setPower(power);
        }
        motorLeft.setPower(0);
        motorRight.setPower(0);

    }


    public void straight2(long ms, double power) throws InterruptedException{
        motorLeft.setPower(power);
        motorRight.setPower(power);
        sleep(ms);
        motorLeft.setPower(0);
        motorRight.setPower(0);
    }

    @Override
    public void runOpMode() throws InterruptedException {

      // set up the hardware devices we are going to use





      motorLeft = hardwareMap.dcMotor.get("motor_1");
      motorRight = hardwareMap.dcMotor.get("motor_2");
      gyro = hardwareMap.gyroSensor.get("gyro");

      motorLeft.setDirection(DcMotor.Direction.REVERSE);

      // wait for the start button to be pressed
      waitForStart();
       /* while(true) {
            telemetry.addData("degrees", gyro.getRotation());
        }*/
        //how to encoder
        motorLeft.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        motorLeft.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);

        turnWithGyro(-30);
        straight(7, 1);
        turnWithGyro(-30);
        straight(2, -1);
        turnWithGyro(-90);
        straight(4, 1);
        turnWithGyro(30);
        straight(2,1);






    }
  }
