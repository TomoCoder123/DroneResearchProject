/*
 * wallfollowing_multirange_onboard.h
 *
 *  Created on: Aug 7, 2018
 *      Author: knmcguire
 */

#ifndef SRC_WALLFOLLOWING_MULTIRANGER_ONBOARD_H_
#define SRC_WALLFOLLOWING_MULTIRANGER_ONBOARD_H_
#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    forward,
    hover,
    turnToFindWall,
    turnToAlignToWall,
    forwardAlongWall,
    rotateAroundWall,
    rotateInCorner,
    findCorner
} StateWF;

typedef struct CommandVel_s{
    float cmdVelX;
    float cmdVelY;
    float cmdAngW;
} CommandVel_t;


int wallFollower(CommandVel_t *commandVel, float frontRange, float sideRange, float currentHeading,
                     int directionTurn, float timeOuter);

void adjustDistanceWall(float distanceWallNew);

void wallFollowerInit(float refDistanceFromWallNew, float maxForwardSpeed_ref);
#endif /* SRC_WALLFOLLOWING_MULTIRANGER_ONBOARD_H_ */
