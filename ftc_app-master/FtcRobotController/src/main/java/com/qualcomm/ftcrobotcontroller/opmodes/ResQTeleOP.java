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
    DcMotor motorWinch;
    DcMotor motorArm;



	/**
	 * Constructor
	 */
	public ResQTeleOP() {
	}


	@Override
	public void init() {
        //if the phones give error can't find motor <name>
        //comment out the line with <name> in it with //
        //this is because the motor isnt set up in the phone,
        //so make sure the ResQ configuration is being used
		motorRight = hardwareMap.dcMotor.get("rightMotor");
		motorLeft = hardwareMap.dcMotor.get("leftMotor");
		motorIntake = hardwareMap.dcMotor.get("intake");
        motorWinch = hardwareMap.dcMotor.get("winch");
        motorArm = hardwareMap.dcMotor.get("arm");
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
	}

	/*
	 * This method will be called repeatedly in a loop
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */


    //double multiplier = 1;


	@Override
	public void loop() {

        double leftPower = gamepad1.left_stick_y;
        double rightPower = gamepad1.right_stick_y;
        // winch is ;left joystick driver 2
        double winchPower = gamepad2.left_stick_y;
        //arm is right joystick driver 2
        double armPower = gamepad2.right_stick_y;
        //intake is driver 2 button "a"
/*
        if (gamepad1.left_bumper) {
            multiplier = 0.5;
        } else if (gamepad1.left_trigger > 0.01) {
            multiplier = 1;
        } else {
            multiplier = 1;
        }
*/


        if (Math.abs(leftPower) < .15) {
            motorLeft.setPower(0);
        } else {
            motorLeft.setPower(leftPower);
        }

        if (Math.abs(rightPower) < .15) {
            motorRight.setPower(0);
        } else {
            motorRight.setPower(rightPower);
        }

        //intake is driver two button "a"
        if (gamepad2.a) {
            motorIntake.setPower(.8);
        } else {
            motorIntake.setPower(0);
        }

        if (Math.abs(winchPower) < .15){
            motorWinch.setPower(0);
        } else {
            motorWinch.setPower(winchPower);
        }

        if(Math.abs(armPower) < .20) {
            motorArm.setPower(0);
        } else {
            motorArm.setPower(armPower);
        }


	}


	@Override
	public void stop() {
	}




}
