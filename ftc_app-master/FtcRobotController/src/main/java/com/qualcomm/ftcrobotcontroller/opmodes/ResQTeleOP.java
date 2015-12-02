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

    //motors
	DcMotor motorLeft;
	DcMotor motorRight;
	DcMotor motorIntake;
    DcMotor motorWinch;
    DcMotor motorArm;

    //servos
    Servo leftZip;
    Servo rightZip;
    Servo servoConveyor;
    Servo rightBoxDoor;
    Servo leftBoxDoor;
    Servo winchPaul; //;) (pawl)

    //hall effect sensors
    DigitalChannel limitWinch;
    DigitalChannel limitArm;

    //booleans for moving servos
    boolean leftZipUp = true;
    boolean rightZipUp = true;
    boolean rightDoorClose = true;
    boolean leftDoorClose = true;
    boolean winchPaulEngage = false;

    //booleans for button states
    boolean lbPressedD1 = false;
    boolean rbPressedD1 = false;
    boolean lbPressedD2 = false;
    boolean rbPressedD2 = false;
    boolean rtPressedD1 = false;

    //values for position servos
    double leftZUp = 0.95;
    double leftZDown = .3;
    double rightZUp = .05;
    double rightZDown = .75;
    double leftDoorOpened = 0.1;
    double leftDoorClosed = 0.6;
    double rightDoorOpened = .62;
    double rightDoorClosed= 0.28;
    double paulEngaged = 0.28;
    double paulDisengaged = 0;


    //Deadzone for motors
    double DEADZONE = 0.15;

    //multipliers fro stopping at limits
    double winchStop = 1;
    double armStop = 1;

    //values to write joystick values to
    double winchPower = 0;
    double armPower = 0;
    double leftPower = 0;
    double rightPower = 0;

	public ResQTeleOP() {
	}

	@Override
	public void init() {
		motorRight = hardwareMap.dcMotor.get("rightMotor");
		motorLeft = hardwareMap.dcMotor.get("leftMotor");
		motorIntake = hardwareMap.dcMotor.get("intake");
        motorWinch = hardwareMap.dcMotor.get("winch");
        motorArm = hardwareMap.dcMotor.get("arm");
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        motorIntake.setDirection(DcMotor.Direction.REVERSE);
        motorArm.setDirection(DcMotor.Direction.REVERSE);
        leftZip = hardwareMap.servo.get("leftZ");
        leftZip.setPosition(leftZUp);
        rightZip = hardwareMap.servo.get("rightZ");
        rightZip.setPosition(rightZUp);
        limitWinch = hardwareMap.digitalChannel.get("winchLimit");
        limitArm = hardwareMap.digitalChannel.get("armLimit");
        servoConveyor = hardwareMap.servo.get("conveyor");
        leftBoxDoor = hardwareMap.servo.get("lDoor");
        rightBoxDoor = hardwareMap.servo.get("rDoor");
        winchPaul = hardwareMap.servo.get("paul");
    }

    @Override
    public void init_loop() {
        leftZip.setPosition(leftZUp);
        rightZip.setPosition(rightZUp);
        leftBoxDoor.setPosition(leftDoorClosed);
        rightBoxDoor.setPosition(rightDoorClosed);
        winchPaul.setPosition(paulDisengaged);
        servoConveyor.setPosition(0.5);
    }

	@Override
	public void loop() {

        //Set the value of joysticks to variables for controlling things by joystick
        //includung drive, arm, winch

        /*
        if(limitWinch.getState()) {
            winchPower = gamepad2.left_stick_y;
        } else if(!limitWinch.getState() && gamepad2.left_stick_y < 0.15) {
            winchPower = gamepad2.left_stick_y;
        } else {
            winchPower = 0;
        }

        if(limitArm.getState()) {
            armPower = gamepad2.right_stick_y;
        } else if(!limitArm.getState() && gamepad2.right_stick_y > 0.2){
            armPower = gamepad2.right_stick_y;
        } else {
            armPower = 0;
        }
        */

        armPower = gamepad2.right_stick_y;
        winchPower = gamepad2.left_stick_y;

        leftPower = gamepad1.left_stick_y;
        rightPower = gamepad1.right_stick_y;


        ////////////////////////////////////////
        ////////////////////////////////////////
        ////////////////////////////////////////

        // Set motors using joystick controls to their respective joystick variable
        // usually <mechanism name>Power

        if (Math.abs(leftPower) < DEADZONE) {
            motorLeft.setPower(0);
        } else {
            motorLeft.setPower(leftPower);
        }

        if (Math.abs(rightPower) < DEADZONE) {
            motorRight.setPower(0);
        } else {
            motorRight.setPower(rightPower);
        }

        if (Math.abs(winchPower) < DEADZONE){
            motorWinch.setPower(0);
        } else {
            motorWinch.setPower(winchPower);
        }

        if(Math.abs(armPower) < DEADZONE) {
            motorArm.setPower(0);
        } else {
            motorArm.setPower(armPower);
        }

        ////////////////////////////////////////
        ////////////////////////////////////////
        ////////////////////////////////////////

        //Use joystick buttons to change booleans controlling servos as
        //well as those that directly control other mechanisms

        //left and right trigger driver 2 control polycord conveyor belt

        if(gamepad1.right_trigger > 0.02 && !rtPressedD1) {
            winchPaulEngage = !winchPaulEngage;
            rtPressedD1 = true;
        } else if(gamepad1.right_trigger < 0.02) {
            rtPressedD1 = false;
        }

        if(gamepad2.left_trigger > 0.02) {
            servoConveyor.setPosition(1);
        } else if(gamepad2.right_trigger > 0.02) {
            servoConveyor.setPosition(0);
        } else {
            servoConveyor.setPosition(.5);
        }

        //right bumper driver 1 is right zipline servo
        if(gamepad1.right_bumper && !rbPressedD1) {
            leftZipUp = !leftZipUp;
            rbPressedD1 = true;
        } else if(!gamepad1.right_bumper) {
            rbPressedD1 = false;
        }

        //left bumper driver 1 is left zipline servo
        if(gamepad1.left_bumper && !lbPressedD1) {
            rightZipUp = !rightZipUp;
            lbPressedD1 = true;
        } else if(!gamepad1.left_bumper) {
            lbPressedD1 = false;
        }

        if(gamepad2.left_bumper && !lbPressedD2) {
            leftDoorClose = !leftDoorClose;
            lbPressedD2 = true;
        } else if(!gamepad2.left_bumper) {
            lbPressedD2 = false;
        }

        if(gamepad2.right_bumper && !rbPressedD2) {
            rightDoorClose = !rightDoorClose;
            rbPressedD2 = true;
        } else if(!gamepad2.right_bumper) {
            rbPressedD2 = false;
        }

        //"a" on driver 2 is intake in, "b" is out
        if (gamepad2.a) {
            motorIntake.setPower(1);
        } else if(gamepad2.b) {
            motorIntake.setPower(-1);
        } else {
            motorIntake.setPower(0);
        }


        ////////////////////////////////////////
        ////////////////////////////////////////
        ////////////////////////////////////////

        //Use booleans to change psoitions of servos

        //Left servo for zipline
        if(leftZipUp) {
            leftZip.setPosition(leftZUp);
        } else {
            leftZip.setPosition(leftZDown);
        }

        //Right servo for zipline
        if(rightZipUp) {
            rightZip.setPosition(rightZUp);
        } else {
            rightZip.setPosition(rightZDown);
        }

        if(leftDoorClose){
            leftBoxDoor.setPosition(leftDoorClosed);
        } else {
            leftBoxDoor.setPosition(leftDoorOpened);
        }

        if(rightDoorClose) {
            rightBoxDoor.setPosition(rightDoorClosed);
        } else {
            rightBoxDoor.setPosition(rightDoorOpened);
        }

        if(winchPaulEngage) {
            winchPaul.setPosition(paulEngaged);
        } else {
            winchPaul.setPosition(paulDisengaged);
        }

	}

	@Override
	public void stop() {
	}
}
