package com.qualcomm.ftcrobotcontroller.opmodes.customcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;


public class LinearTutorial extends LinearOpMode {
  private final int BACKUP_TIME = 1000;
  private final int TURN_TIME = 250;

  final static int ENCODER_CPR = 1440;
  final static double GEAR_RATIO = 2;
  final static int WHEEL_DIAMETER = 4;
  final static int DISTANCE = 24;
  final static double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
  final static double ROTATIONS = DISTANCE / CIRCUMFERENCE;
  final static double COUNTS = ENCODER_CPR * ROTATIONS * GEAR_RATIO;

  @Override
  public void runOpMode() throws InterruptedException {
    DcMotor rightMotor = hardwareMap.dcMotor.get("right_drive");
    DcMotor leftMotor = hardwareMap.dcMotor.get("left_drive");
    TouchSensor touchSensor = hardwareMap.touchSensor.get("sensor_touch");
    OpticalDistanceSensor opticalDistanceSensor = hardwareMap.opticalDistanceSensor.get
        ("sensor_EOPD");

    leftMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
    rightMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);

    leftMotor.setTargetPosition((int) COUNTS);
    rightMotor.setTargetPosition((int) COUNTS);

    leftMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
    rightMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

    leftMotor.setPower(0.5);
    rightMotor.setPower(0.5);

    rightMotor.setDirection(DcMotor.Direction.REVERSE);

    waitForStart();

    rightMotor.setPower(1);
    leftMotor.setPower(1);

    sleep(2000);

    rightMotor.setPower(0);
    leftMotor.setPower(0);

    for (int i = 0; i < 4; i++) {
      rightMotor.setPower(1);
      leftMotor.setPower(1);

      sleep(3000);

      rightMotor.setPower(0);
      leftMotor.setPower(0.5);

      sleep(1500);
    }

    if (touchSensor.isPressed()) {
      rightMotor.setPower(0);
      leftMotor.setPower(0);
    } else {
      rightMotor.setPower(1);
      leftMotor.setPower(1);
    }

    telemetry.addData("isPressed", String.valueOf(touchSensor.isPressed()));
  }
}
