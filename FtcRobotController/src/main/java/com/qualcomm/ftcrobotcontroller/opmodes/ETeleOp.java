/*
  Copyright (c) 2015 Edina Terminators Robotics
  http://etrobotics.org/
  GitHub: https://github.com/EdinaTerminators/
  
  Developed by Richik SC and ...
*/
package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class ETeleOp extends OpMode {
  
  DcMotor motorRight;
  DcMotor motorLeft;
  
  public ETeleOp() {
    
    
    
  }
  
  @Override // Overrides superclass OpMode method init
  public void init() {
    
    motorRight = hardwareMap.dcMotor.get("motor_2"); // Change to whatever motor name given
    motorLeft = hardwareMap.dcMotor.get("motor_1"); //  ''
    motorLeft.setDirection(DcMotor.Direction.REVERSE);
    
  }
  
  @Override // 26
  public void loop() {
    
    /*
      Gamepad sticks:
      Throttle / Y direction:
        Full up = -1
        Full down = 1
      Reversing needed, because forward as positive value makes sense.
      Steering / X direction:
        Full left = -1
        Full right = 1
    */
    
    float throttle = -gamepad1.left_stick_y;
    float steering = gampead1.left_stick_x;
    float right = throttle - direction;
    float left = throttle + direction;
    
    right = Range.clip(right, -1, 1); // Clip values so they do not exceed 1 or -1
    left = Range.clip(left, -1, 1); // ''
    
    right = (float)scaleInput(right);
    left = (float)scaleInput(left);
    
    motorRight.setPower(right);
    motorLeft.setPower(left);
    
  }
  
  @Override // 26
  public void stop() {
    telemetry.addData("Stop", "Robot Stopped");
  }
  /* Copied from K9TeleOp.java
  Copyright (c) 2014  Qualcomm Technologies Inc
  */
  private double scaleInput(double dVal) {
    double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
				0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };
			
		// get the corresponding index for the scaleInput array.
		int index = (int) (dVal * 16.0);

		// index should be positive.
		if (index < 0) {
			index = -index;
		}

		// index cannot exceed size of array minus 1.
		if (index > 16) {
			index = 16;
		}

		// get value from the array.
		double dScale = 0.0;
		if (dVal < 0) {
			dScale = -scaleArray[index];
		} else {
			dScale = scaleArray[index];
		}

		// return scaled value.
		return dScale;
  }
}