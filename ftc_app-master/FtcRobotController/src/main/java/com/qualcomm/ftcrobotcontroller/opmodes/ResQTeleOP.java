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
import com.qualcomm.robotcore.robocol.Telemetry;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class ResQTeleOP extends OpMode {

    //countdown timer
    ElapsedTime countdown = new ElapsedTime();

    //motors
	DcMotor motorLeft;
	DcMotor motorRight;
	DcMotor motorIntake;
    DcMotor motorWinch;
    DcMotor motorArm;
    DcMotor motorLeftTwo;

    //servos
    Servo leftZip;
    Servo rightZip;
    Servo servoConveyor;
    Servo rightBoxDoor;
    Servo leftBoxDoor;
    Servo winchPaul; //;) (pawl)

    //modules
    LegacyModule legacy;
    DeviceInterfaceModule cdim;

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
    boolean yPressedD1 = false;

    //values for position servos
    double leftZUp = 0.95;
    double leftZDown = .3;
    double rightZUp = .05;
    double rightZDown = .75;
    double leftDoorOpened = 0.1;
    double leftDoorClosed = 0.72;
    double rightDoorOpened = .62;
    double rightDoorClosed= 0.28;
    double paulEngaged = 0.31;
    double paulDisengaged = 0.1;


    //Deadzone for motors
    double DEADZONE = 0.15;

    //boolean to chnage the front of the robot
    boolean intakeFront = true;

    //values to write joystick values to
    double winchPower = 0;
    double armPower = 0;
    double leftPower = 0;
    double rightPower = 0;

    String legacyStatus = "CONNECTED";
    String cdimStatus = "CONNECTED";

	public ResQTeleOP() {
	}

	@Override
	public void init() {
		motorRight = hardwareMap.dcMotor.get("rightMotor");
		motorLeft = hardwareMap.dcMotor.get("leftMotor");
		motorIntake = hardwareMap.dcMotor.get("intake");
        motorWinch = hardwareMap.dcMotor.get("winch");
        motorArm = hardwareMap.dcMotor.get("arm");
        motorLeftTwo = hardwareMap.dcMotor.get("l2");
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        motorLeftTwo.setDirection(DcMotor.Direction.REVERSE);
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
        legacy = hardwareMap.legacyModule.get("leg");
        cdim = hardwareMap.deviceInterfaceModule.get("cdim");
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

    long startTime = 0;

    @Override
    public void start() {
        startTime = System.nanoTime();
        countdown.reset();
    }

	@Override
	public void loop() {

        //COuntdown to end of match

        telemetry.addData("TIME REMAINING", (int)(120 - countdown.time()) + "Seconds");

        //Set the value of joysticks to variables for controlling things by joystick
        //includung drive, arm, winch


        if(limitWinch.getState()) {
            winchPower = gamepad2.left_stick_y;
        } else if(!limitWinch.getState() && gamepad2.left_stick_y < -0.15) {
            winchPower = gamepad2.left_stick_y;
        } else {
            winchPower = 0;
        }

        if(limitArm.getState()) {
            armPower = gamepad2.right_stick_y;
        } else if(!limitArm.getState() && gamepad2.right_stick_y < -0.2){
            armPower = gamepad2.right_stick_y;
        } else {
            armPower = 0;
        }
<<<<<<< HEAD
=======

>>>>>>> origin/master


        if(gamepad1.y && !yPressedD1) {
            intakeFront = !intakeFront;
            yPressedD1 = true;
        } else if(!gamepad1.y) {
            yPressedD1 = false;
        }

        //armPower = gamepad2.right_stick_y;
        //winchPower = gamepad2.left_stick_y;

        leftPower = gamepad1.left_stick_y;
        rightPower = gamepad1.right_stick_y;


        ////////////////////////////////////////
        ////////////////////////////////////////
        ////////////////////////////////////////

        // Set motors using joystick controls to their respective joystick variable
        // usually <mechanism name>Power



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
/*
        if(gamepad1.right_trigger > 0.02 && !rtPressedD1) {
            winchPaulEngage = !winchPaulEngage;
            rtPressedD1 = true;
        } else if(gamepad1.right_trigger < 0.02) {
            rtPressedD1 = false;
        }
*/



        if (intakeFront) {
            intake_front();
        } else {
            arm_front();
        }

        if(gamepad1.left_trigger > 0.02) {
            winchPaulEngage = true;
        } else if (gamepad1.right_trigger > 0.02) {
            winchPaulEngage = false;
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

        /////////////////////////////////
        /////////////////////////////////
        /////////////////////////////////

        //stop code after 2 minutes
/*
        if(System.nanoTime() - startTime > 120E9) {
            FtcOpModeRegister.opModeManager.stopActiveOpMode();
            return;
        }
        */

        telemetry.addData("legacy", legacy.getConnectionInfo());
        telemetry.addData("cdim", cdim.getConnectionInfo());

	}

	@Override
	public void stop() {
	}

    public void intake_front() {

        if (Math.abs(rightPower) < DEADZONE) {
            setLeftPower(0);
        } else {
            setLeftPower(-rightPower);
        }

        if (Math.abs(leftPower) < DEADZONE) {
            motorRight.setPower(0);
        } else {
            motorRight.setPower(-leftPower);
        }

    }

    public void arm_front() {
        if (Math.abs(leftPower) < DEADZONE) {
            setLeftPower(0);
        } else {
            setLeftPower(leftPower);
        }

        if (Math.abs(rightPower) < DEADZONE) {
            motorRight.setPower(0);
        } else {
            motorRight.setPower(rightPower);
        }
    }

    public void setLeftPower(double power) {
        motorLeft.setPower(power);
        motorLeftTwo.setPower(power);
    }
}



