package com.qualcomm.ftcrobotcontroller.opmodes.customcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;

public class ETAutonomousBase extends LinearOpMode {

  public static final double DIST_FACTOR = 3.4;
  public static final double MOVE_FIRST_DISTANCE = 100 / DIST_FACTOR;
  public static final double MOVE_POWER = -0.25; // TODO figure out why this value is negative
  public static final double MOVE_SECOND_DISTANCE = 36 / DIST_FACTOR;
  public static final double REVERSE_FIRST_DISTANCE = -27 / DIST_FACTOR;
  public static final double TURN_COUNTS = 18 / DIST_FACTOR;

  public static final double MAIN_RAISE_UP = 0.52;
  public static final double MAIN_RAISE_DOWN = 0.95;
  public static final double CLIMBERS_UP = 0;

  final static int ENCODER_CPR = 1440;            // Encoder counts per revolution
  final static double GEAR_RATIO = 1;             // Gear ratio
  final static double WHEEL_DIAMETER = 2.625;     // Diameter of wheel
  final static double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
  final static int ERROR_THRESHOLD = 10;

  // Declare hardware devices
  protected DcMotor right;
  protected DcMotor left;
  protected Servo mainRaise;
  protected Servo climberL;
  protected Servo climberR;

  // Variables
  protected static boolean isTargetSet = false;
  protected static double counts = 0;
  protected static int stage;
  protected boolean loopBreaker;

  // CONTROL STAGES
  final static int STAGE_MOVE_FIRST = 1;
  final static int STAGE_CONTINUE_MOVE_FIRST = 2;
  final static int STAGE_MOVE_SECOND = 3;
  final static int STAGE_CONTINUE_MOVE_SECOND = 4;
  final static int STAGE_TURN_FIRST = 5;
  final static int STAGE_CONTINUE_TURN_FIRST = 6;
  final static int STAGE_REVERSE_FIRST = 7;
  final static int STAGE_CONTINUE_REVERSE_FIRST = 8;
  final static int STAGE_STOP = 99;
  final static int STAGE_BREAK_LOOP = 100;

  public void etInit() throws InterruptedException {
    right = hardwareMap.dcMotor.get("Right");
    left = hardwareMap.dcMotor.get("Left");
    if(left != null) {
      left.setDirection(DcMotor.Direction.REVERSE);
    }

    mainRaise = hardwareMap.servo.get("MainRaise");
    climberR = hardwareMap.servo.get("ClimberR");
    climberL = hardwareMap.servo.get("ClimberL");
    if(climberL != null) {
      climberL.setDirection(Servo.Direction.REVERSE);
    }

    // Reset all variables to initial value
    stage = STAGE_MOVE_FIRST; // Stage = first stage
    loopBreaker = false; // Loop is not broken
    counts = 0; // Counts is 0

    pusherUp();
    resetClimbers();

  }
  public void etSetup() throws InterruptedException {
    pusherDown();
    resetEncoders();
  }
  public void etLoop() throws InterruptedException {
    
  }

  public void etBreakLoop() throws InterruptedException {
    loopBreaker = true;
  }

  @Override
  public void runOpMode()  throws InterruptedException {
    etInit();
    waitForStart();
    etSetup();
    //int i = 0;
    while(opModeIsActive()) {
      if(loopBreaker) {
        break;
      }
      etLoop();
      waitOneFullHardwareCycle();
    }
  }

  protected double getCountsForDistance(double distance) {
    double rotations = distance / CIRCUMFERENCE;
    return ENCODER_CPR * rotations * GEAR_RATIO;
  }

  protected boolean hasArrived(double leftCtsTarget, double rightCtsTarget) {

    if (hasArrivedLeft(leftCtsTarget) && hasArrivedRight(rightCtsTarget)) {
      isTargetSet = false;
      return true;
    }
    return false;

  }

  protected void stopRobot() throws InterruptedException {
    resetEncoders();
    drivePower(0d, 0d);
  }

  protected boolean hasArrivedLeft(double cts) {

    telemetry.addData("Left Position", left.getCurrentPosition());
    double absCts = Math.abs(cts);
    int absPosition = Math.abs(left.getCurrentPosition());
    int errorMargin = Math.abs(absPosition - ((int) absCts));

    if (errorMargin < ERROR_THRESHOLD)
      return true;

    return false;
  }

  protected boolean hasArrivedRight(double cts) {

    telemetry.addData("Right Position", right.getCurrentPosition());
    double absCts = Math.abs(cts);
    int absPosition = Math.abs(right.getCurrentPosition());
    int errorMargin = Math.abs(absPosition - ((int) absCts));

    if (errorMargin < ERROR_THRESHOLD)
      return true;

    return false;

  }

  protected boolean hasEncoderResetLeft() {

    if(left != null) {
      if(left.getCurrentPosition() == 0) {
        return true;
      }
    }
    return false;

  }

  protected boolean hasEncoderResetRight() {

    if(right != null) {
      if(right.getCurrentPosition() == 0) {
        return true;
      }
    }
    return false;
  }

  protected void resetEncoderLeft() {

    if(left != null) {
      left.setMode(DcMotorController.RunMode.RESET_ENCODERS);
    }

  }

  protected void resetEncoderRight() {

    if(right != null) {
      right.setMode(DcMotorController.RunMode.RESET_ENCODERS);
    }

  }

  protected boolean hasEncoderReset() {
    if(hasEncoderResetLeft() && hasEncoderResetRight()) {
      return true;
    }
    return false;

  }

  protected void resetEncoders() {

    resetEncoderLeft();
    resetEncoderRight();

  }

  protected boolean encoderDrive(
      double leftPower,
      double rightPower,
      double leftTarget,
      double rightTarget)
  {
    runUsingEncoders();
    drivePower(leftPower, rightPower);
    if(hasArrived(leftTarget, rightTarget)) {
      resetEncoders();
      drivePower(0d, 0d);
      return true;
    }
    return false;
  }

  protected void drivePower(double leftPower, double rightPower) {
    drivePowerLeft(leftPower);
    drivePowerRight(rightPower);
  }

  protected void drivePowerLeft(double power) {
    if(left != null) {
      left.setPower(power);
    }
  }

  protected void drivePowerRight(double power) {
    if(right != null) {
      right.setPower(power);
    }
  }

  protected void runUsingEncoders() {
    if (left != null && right != null) {
      left.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
      right.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
    }
  }

  protected void resetClimbers() {
    if(climberL != null && climberR != null) {
      climberL.setPosition(CLIMBERS_UP);
      climberR.setPosition(CLIMBERS_UP);
    }
  }

  protected void pusherDown() {
    if(mainRaise != null) {
      mainRaise.setPosition(MAIN_RAISE_DOWN);
      telemetry.addData("Pusher Position", "DOWN");
    }
  }

  protected void pusherUp() {
    if(mainRaise != null) {
      mainRaise.setPosition(MAIN_RAISE_UP);
      telemetry.addData("Pusher Position", "UP");
    }
  }

}