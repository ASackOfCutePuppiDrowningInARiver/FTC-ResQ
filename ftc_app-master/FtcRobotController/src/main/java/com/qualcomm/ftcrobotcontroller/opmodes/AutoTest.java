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
import com.qualcomm.robotcore.hardware.Servo;

/**
 * A simple example of a linear op mode that will approach an IR beacon
 */
public class AutoTest extends LinearOpMode {


    DcMotor motorLeft;
    //DcMotor motorRight;
    //GyroSensor gyro;
    DcMotorController lel;
    Servo kek;



    @Override
    public void runOpMode() throws InterruptedException {

        // set up the hardware devices we are going to use
        motorLeft = hardwareMap.dcMotor.get("leftMotor");
        //motorRight = hardwareMap.dcMotor.get("rightMotor");
        //gyro = hardwareMap.gyroSensor.get("gyro");
        lel = hardwareMap.dcMotorController.get("MC0");
        //motorLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        //motorRight.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        //functions.initializerobot();
        // wait for the start button to be pressed
        //lel.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);
        lel.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);
        waitForStart();
        //start




    }
}
