package com.qualcomm.ftcrobotcontroller.opmodes.customcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


public class TeleOp extends OpMode {
  public static final double MAIN_RAISE_UP = 0.55;
  public static final int CLIMBER_UP = 0;
  public static final double MAIN_RAISE_DOWN = 0.95;
  public static final double CLIMBER_DOWN = 0.8;
  private DcMotor right;
  private DcMotor left;
  private Servo mainRaise;
  private Servo climberR;
  private Servo climberL;
  private DcMotor tapeMeasure;
  private DcMotor hook;

  @Override
  public void init() {
    right = hardwareMap.dcMotor.get("Right");
    left = hardwareMap.dcMotor.get("Left");
    tapeMeasure = hardwareMap.dcMotor.get("TapeMeasure");
    hook = hardwareMap.dcMotor.get("Hook");
    mainRaise = hardwareMap.servo.get("MainRaise");
    climberR = hardwareMap.servo.get("ClimberR");
    climberL = hardwareMap.servo.get("ClimberL");
    left.setDirection(DcMotor.Direction.REVERSE);
    climberL.setDirection(Servo.Direction.REVERSE);
    //backRight.setDirection(DcMotor.Direction.REVERSE);
    //servoM.setPosition(0.6);
    mainRaise.setPosition(MAIN_RAISE_UP);
    climberR.setPosition(CLIMBER_UP);
    climberL.setPosition(CLIMBER_UP);
  }

  @Override
  public void loop() {
    float leftY = -gamepad1.left_stick_y;
    float rightY = -gamepad1.right_stick_y;
    boolean toggled = false;
    boolean tapeIsHiPower = false;
    double factor = 0.25;
    //double servoPositionR = 0;
    //double servoPositionL = 0;

    if(gamepad1.right_bumper || gamepad1.left_bumper)
      toggled = true;
    telemetry.addData("Inverted Controls", (toggled ? "On":"Off"));
    if(toggled) {
        leftY = gamepad1.left_stick_y;
        rightY = gamepad1.right_stick_y;
    }

    if(gamepad2.right_trigger > 0.25) {
      tapeIsHiPower = true;
    } else {
      tapeIsHiPower = false;
    }

    if(gamepad2.right_stick_y > 0.25) {
      double power = gamepad2.right_stick_y;
      Range.clip(power, 0, 1);
      if(tapeIsHiPower) {
        tapeMeasure.setPower(0.5);
        hook.setPower(0.5);
      } else {
        tapeMeasure.setPower(0.1);
        hook.setPower(0.1);
      }

    }
    if(gamepad2.right_stick_y < -0.25) {
      double power = gamepad2.right_stick_y;
      Range.clip(power, 0, -1);
      tapeMeasure.setPower(-0.1);
      hook.setPower(-0.1);
    }
    if( -0.25 <= gamepad2.right_stick_y && gamepad2.right_stick_y <= 0.25) {
      tapeMeasure.setPowerFloat();
      hook.setPowerFloat();
    }

    if(gamepad1.right_trigger > 0.25)
      factor = gamepad1.right_trigger;

    telemetry.addData("Power Factor", factor);
    //telemetry.addData("ServoR", servoPositionR);
    //telemetry.addData("ServoL", servoPositionL);
    right.setPower(rightY * factor);
    left.setPower(leftY * factor);

    if(gamepad2.left_bumper) {
      mainRaise.setPosition(MAIN_RAISE_DOWN);
    }
    if(gamepad2.right_bumper) {
      mainRaise.setPosition(MAIN_RAISE_UP);
    }

    if(gamepad2.x) {
      climberR.setPosition(CLIMBER_DOWN);
    }
    if(gamepad2.a) {
      climberR.setPosition(CLIMBER_UP);
    }
    if(gamepad2.dpad_down) {
      climberL.setPosition(CLIMBER_UP);
    }
    if(gamepad2.dpad_up) {
      climberL.setPosition(CLIMBER_DOWN);
    }
  }
}
