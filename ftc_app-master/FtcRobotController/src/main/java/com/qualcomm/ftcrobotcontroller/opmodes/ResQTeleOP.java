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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class ResQTeleOP extends OpMode {


	DcMotor motorLeft;
	DcMotor motorRight;
	DcMotor motorIntake;

    FUNctions functions = new FUNctions();



	/**
	 * Constructor
	 */
	public ResQTeleOP() {
	}


	@Override
	public void init() {

		motorRight = hardwareMap.dcMotor.get("rightMotor");
		motorLeft = hardwareMap.dcMotor.get("leftMotor");
		motorIntake = hardwareMap.dcMotor.get("intake");
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
// hola BOB! says hola HAHAHAHAHAHAHA


	}

	/*
	 * This method will be called repeatedly in a loop
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */



	@Override
	public void loop() {

		double leftPower = gamepad1.left_stick_y;
		double rightPower = gamepad1.right_stick_y;
       // boolean intakeButton = gamepad1.a;

        //functions.tankDrive(motorLeft, motorRight, gamepad1.left_stick_y, gamepad1.right_stick_y);


		if (Math.abs(leftPower) < .15) {
			motorLeft.setPower(0);
		}
		else {
			motorLeft.setPower(leftPower);
		}

		if (Math.abs(rightPower) < .15) {
			motorRight.setPower(0);
		}
		else  {
			motorRight.setPower(rightPower);
		}

	/*	if (intakeButton == true) {
			motorIntake.setPower(1);
		}
        else {
            motorIntake.setPower(0);
        }
*/

	}


	@Override
	public void stop() {
	}




}
