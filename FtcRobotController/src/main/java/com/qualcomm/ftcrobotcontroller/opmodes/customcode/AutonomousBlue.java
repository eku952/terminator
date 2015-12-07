/*
 * Copyright (c) 2015 Edina Terminators Robotics
 *
 * This software is distributed under the MIT License. The license text can be read in full at /LICENSE.txt
 * Authored by:
 * Luke Langefels <https://eku952@github.com>
 * Richik Sinha Choudhury <https://richiksc@github.com>
 */

package com.qualcomm.ftcrobotcontroller.opmodes.customcode;

public class AutonomousBlue extends ETAutonomousBase {

  private static double lCounts;
  private static double rCounts;

  @Override
  public void etLoop() throws InterruptedException {

    telemetry.addData("stage", stage);

    switch (stage) {
      case STAGE_MOVE_FIRST:
      {
        counts = getCountsForDistance(MOVE_FIRST_DISTANCE);
        telemetry.addData("STAGE", "STAGE_MOVE_FIRST");
        if(hasEncoderReset()) {
          encoderDrive(MOVE_POWER, MOVE_POWER, counts, counts);
          stage = STAGE_CONTINUE_MOVE_FIRST;
        }
        break;
      }

      case STAGE_CONTINUE_MOVE_FIRST: {
        if (encoderDrive(MOVE_POWER, MOVE_POWER, counts, counts)) {
          stage = STAGE_REVERSE_FIRST;
        }
        break;
      }

      case STAGE_REVERSE_FIRST: {
        counts = getCountsForDistance(REVERSE_FIRST_DISTANCE);
        if(hasEncoderReset()) {
          encoderDrive(MOVE_POWER * -1, MOVE_POWER * -1, counts, counts);
          stage = STAGE_CONTINUE_REVERSE_FIRST;
        }
        break;
      }

      case STAGE_CONTINUE_REVERSE_FIRST: {
        if(encoderDrive(MOVE_POWER * -1, MOVE_POWER * -1, counts, counts)) {
          stage = STAGE_TURN_FIRST;
        }
        break;
      }

      case STAGE_TURN_FIRST: {
        // turns right
        lCounts = getCountsForDistance(TURN_COUNTS);
        rCounts = lCounts * -1;
        if(hasEncoderReset()) {
          encoderDrive(MOVE_POWER, MOVE_POWER * -1, lCounts, rCounts);
          stage = STAGE_TURN_FIRST;
        }
        break;
      }

      case STAGE_CONTINUE_TURN_FIRST: {
        if(encoderDrive(MOVE_POWER, MOVE_POWER * -1, lCounts, rCounts)) {
          pusherUp();
          stage = STAGE_MOVE_SECOND;
        }
        break;
      }

      case STAGE_MOVE_SECOND: {
        counts = getCountsForDistance(MOVE_SECOND_DISTANCE);
        if(hasEncoderReset()) {
          encoderDrive(MOVE_POWER, MOVE_POWER, counts, counts);
          stage = STAGE_CONTINUE_MOVE_SECOND;
        }
        break;
      }

      case STAGE_CONTINUE_MOVE_SECOND: {
        if(encoderDrive(MOVE_POWER, MOVE_POWER, counts, counts)) {
          stage = STAGE_STOP;
        }
        break;
      }

      case STAGE_STOP: {
        stopRobot();
        stage = STAGE_BREAK_LOOP;
        break;
      }

      case STAGE_BREAK_LOOP: {
        if(hasEncoderReset()) {
          etBreakLoop();
        }
        break;
      }

    }
  }
}
