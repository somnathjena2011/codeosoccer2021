#pragma once
#include "skills.h"

namespace MyStrategy
{

	// Naive example for attacker
	void attacker(BeliefState *state, int botID)
	{
		Vec2D awayGoal(HALF_FIELD_MAXX, 0);
		Vec2D ballPoint(state->ballPos.x, state->ballPos.y);
		//Go to ball if ball is far
		//Take ball to the goal
		//strips currStrip = whichStrip(state->ballPos.x, state->ballPos.y);

		//Try crossing to striker if ball in corner
		//if (state->ballPos.x > 4200 - 4 * BOT_RADIUS) {
		//	if (currStrip == TOP_STRIP && state->homePos[botID].y < state->ballPos.y) {
		//	if (currStrip == TOP_STRIP && state->homePos[botID].y < state->ballPos.y) {
		//		shootForAssist(state, botID);
		//		//Spin(botID, -MAX_BOT_OMEGA / 5);
		//		return;
		//	}
		//	else if (currStrip == BOTTOM_STRIP && state->homePos[botID].y > state->ballPos.y) {
		//		shootForAssist(state, botID);
		//		//Spin(botID, MAX_BOT_OMEGA / 5);
		//		return;
		//	}
		//}
		/*
		float botAngle = state->homeAngle[botID];
		Vec2D displacement = state->ballPos - state->homePos[botID];
		float ballAngle = atan2( displacement.y , displacement.x );*/


		/*******************************************pretty unpredictable code**************************************************/

		/*if (Vec2D::distSq(state->homePos[botID], state->ballPos) < 4 * BOT_BALL_THRESH * BOT_BALL_THRESH ) {
		if (Vec2D::distSq(state->homePos[botID], awayGoal) > 4 * DBOX_WIDTH * DBOX_WIDTH)
		{
		print("Move Towards goal");
		ballPoint = predictBallPos(state, botID);
		GoToPoint(botID, state, ballPoint, PI / 2, true, false);
		}
		else
		{
		print("Spin for goal");
		shootForGoal(state, botID);
		}
		}
		else
		GoToPoint(botID, state, ballPoint, 0, true, true);*/
		int THRESHOLD_VELOCITY_X = 60;

		if (state->homeVel[botID].x < -THRESHOLD_VELOCITY_X && state->homePos[botID].x > state->ballPos.x && state->ballPos.x < (-HALF_FIELD_MAXX + 2 * DBOX_WIDTH) && whichStrip(state->ballPos.x, state->ballPos.y) == MIDDLE_STRIP && state->homePos[botID].x < (-HALF_FIELD_MAXX + 3 * DBOX_WIDTH)) {
			Stop(botID);
			return;
		}

		if (Vec2D::distSq(state->homePos[botID], state->ballPos) < 4 * BOT_BALL_THRESH * BOT_BALL_THRESH) {
			if (Vec2D::distSq(state->homePos[botID], awayGoal) > 8 * DBOX_WIDTH * DBOX_WIDTH) {
				//GoToBall
				//GoToPoint(botID, state, awayGoal, PI / 2, true, false);
				dodge(state, botID, Vec2D(OPP_GOAL_X, 0), 0, true);
			}
			else
				shootForGoal(state, botID);
		}
		else
			GoToPoint(botID, state, ballPoint, 0, true, true);

		//shoot(botID, state, Vector2D<float>(awayGoal.x, awayGoal.y));

		//Vec2D awayGoal(HALF_FIELD_MAXX, 0);
		//Vec2D ballPoint(state->ballPos.x, state->ballPos.y);

		
		////Go to ball if ball is far
		////Take ball to the goal
		//strips currStrip = whichStrip(state->ballPos.x, state->ballPos.y);

		////Try crossing to striker if ball in corner
		//if (state->ballPos.x > 4200 - 4 * BOT_RADIUS) {
		//	if (currStrip == TOP_STRIP && state->homePos[botID].y < state->ballPos.y) {
		//		shootForAssist(state, botID);
		//		//Spin(botID, -MAX_BOT_OMEGA / 5);
		//		return;
		//	}
		//	else if (currStrip == BOTTOM_STRIP && state->homePos[botID].y > state->ballPos.y) {
		//		shootForAssist(state, botID);
		//		//Spin(botID, MAX_BOT_OMEGA / 5);
		//		return;
		//	}
		//}
		//
		//float botAngle = state->homeAngle[botID];
		//Vec2D displacement = state->ballPos - state->homePos[botID];
		//float ballAngle = atan2( displacement.y , displacement.x );


		///*******************************************pretty unpredictable code**************************************************/

		///*if (Vec2D::distSq(state->homePos[botID], state->ballPos) < 4 * BOT_BALL_THRESH * BOT_BALL_THRESH ) {
		//if (Vec2D::distSq(state->homePos[botID], awayGoal) > 4 * DBOX_WIDTH * DBOX_WIDTH)
		//{
		//print("Move Towards goal");
		//ballPoint = predictBallPos(state, botID);
		//GoToPoint(botID, state, ballPoint, PI / 2, true, false);
		//}
		//else
		//{
		//print("Spin for goal");
		//shootForGoal(state, botID);
		//}
		//}
		//else 
		//GoToPoint(botID, state, ballPoint, 0, true, true);*/
		//if (Vec2D::distSq(state->homePos[botID], state->ballPos) < 4 * BOT_BALL_THRESH * BOT_BALL_THRESH 
		//	&& abs(ballAngle - botAngle) < PI / 3 
		//	&& state->ballPos.x  > state->homePos[botID].x) {
		//	if (Vec2D::distSq(state->homePos[botID], awayGoal) > 8 * DBOX_WIDTH * DBOX_WIDTH)
		//	{
		//		print("aligned go for goal");
		//		GoToPointStraight(botID, state, awayGoal, PI / 2, true, true);
		//	}
		//	else
		//	{
		//		print("near goal and aligned shoot!!");
		//		shootForGoal(state, botID);
		//	}
		//}
		//else if (state->ballPos.x > state->homePos[botID].x)
		//{
		//	//ballPoint = predictBallPos(state, botID);
		//	GoToPoint(botID, state, ballPoint, 0, false, false);
		//}
		//else
		//{
		//	//print("defender doing his job, stay");
		//	GoToPoint(botID, state, origin, PI / 2, false, true);
		//}
		//else if(Vec2D::distSq(state->homePos[botID], state->ballPos) < 4 * BOT_BALL_THRESH * BOT_BALL_THRESH)
		//{
		//	print("near ball, not aligned go to ball");
		//	GoToBall(botID, state, true);
		//}
		//else if (state->ballPos.x > state->homePos[botID].x)
		//{
		//	print("Ball safe to go GO!!");
		//	ballPoint = predictBallPos(state, botID);
		//	GoToPoint(botID, state, ballPoint, 0, true, true);
		//}
		//else if (state->ballPos.x < state->homePos[botID].x && Vec2D::dist(state->ballPos, state->homePos[1]) > 4 * BOT_BALL_THRESH)
		//{
		//	print("Ball not with defender, try to take ball");
		//	GoToBall(botID, state, true);
		//}
		//else
		//{
		//	print("defender doing his job, stay");
		//	GoToPoint(botID, state, origin, PI / 2, true, true);
		//}

		//shoot(botID, state, Vector2D<float>(awayGoal.x, awayGoal.y));
	}
}