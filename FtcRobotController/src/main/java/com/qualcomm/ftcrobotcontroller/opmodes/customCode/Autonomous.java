package com.qualcomm.ftcrobotcontroller.opmodes.customCode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

public class Autonomous extends ETBaseOpMode {
  private DcMotor right;
  private DcMotor left;

  final static int ENCODER_CPR = 1440;    //encoder counts per revolution
  final static double GEAR_RATIO = 1;     //gear ratio
  final static double WHEEL_DIAMETER = 2.625;     //diameter of wheel
  final static double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
  final static int DISTANCE = 5;
  final static double ROTATIONS = DISTANCE / CIRCUMFERENCE;
  final static double COUNTS = ENCODER_CPR * ROTATIONS * GEAR_RATIO;
  final static int ERROR_THRESHOLD = 10;

  @Override
  public void etSetup() throws InterruptedException {
    right = hardwareMap.dcMotor.get("Right");
    left = hardwareMap.dcMotor.get("Left");
    right.setDirection(DcMotor.Direction.REVERSE);

    right.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
    left.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
    telemetry.addData("Reset Encoders", "Done");
    right.setTargetPosition((int) COUNTS);
    left.setTargetPosition((int) COUNTS);

    right.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
    left.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
    telemetry.addData("Running to Target", "Started");

    telemetry.addData("Motor Target", COUNTS);
    telemetry.addData("Left Position", left.getCurrentPosition());
    telemetry.addData("Right Position", right.getCurrentPosition());

    right.setPower(0.5);
    left.setPower(0.5);
    telemetry.addData("Power Set", "true");
    telemetry.addData("Motor Power", "0.5");

    telemetry.addData("Motor Target", COUNTS);
    telemetry.addData("Left Position", left.getCurrentPosition());
    telemetry.addData("Right Position", right.getCurrentPosition());
  }
  
  @Override
  public void etLoop() throws InterruptedException {
    telemetry.addData("Motor Target", COUNTS);
    telemetry.addData("Left Position", left.getCurrentPosition());
    telemetry.addData("Right Position", right.getCurrentPosition());
    int error = Math.abs(right.getCurrentPosition()) - (int) COUNTS;

    if (error < ERROR_THRESHOLD) {
        right.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        left.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        telemetry.addData("Final Reset", "Done");
        break;
    }
  }
}
