#pragma once
#include "skills.h"

/*
	Vec2D can be used to make variables that can store points as (x,y) and provide utility functions.
	Vec2D point1;
	Vec2D point2;
	point1.x = HALF_FIELD_MAXX;
	point1.y = 0;
	similarly for point2 can be specified
	Some utility functions:
	These will appear automatically as you type Vec2D::
	float angle = Vec2D::angle(point1,point2);
	int distance = Vec2D::distSq(point1,point2);
*/

namespace MyStrategy
{
	Vec2D homeGoal(-4200, 0);


	// Naive Example for GoalKeeper
	void goalkeeper(BeliefState *state, int botID)
	{	
		int safeX = -4250;
		if (botID == 2)
		{
			safeX = -3650;
		}
		int dist = Vec2D::distSq(state->ballPos, homeGoal);
		Vec2D safepoint(-4000, 0);

		Vec2D pos = state->homePos[botID];
		Vec2D dest = Vec2D(pos);
		Vec2D ballpos = state->ballPos;
		int THRESH_VEL = 100;
		// conditions for clearing the ball
		if (dist < DBOX_WIDTH*DBOX_WIDTH * 2 && (state->ballVel.x > -THRESH_VEL) && !(botID == 2 && abs(state->ballPos.y) < OUR_GOAL_Y)) {
			//print("Shoot!!!");
			GoToBall(botID, state, false);
			save_goal(botID, state, Vector2D<float>(state->ballPos.x, state->ballPos.y));
			return;
		}
		dest.x = safeX;

		if ( botID == 0 && Vec2D::dist(state->ballPos, state->homePos[botID]) < DBOX_WIDTH * 1.5 )
		{
			dest.y = state->ballPos.y;
		}
		else
		{
			dest.y = rayCastY(state, botID);
		}

		if (dest.y > OUR_GOAL_MAXY) {
			dest.y = OUR_GOAL_MAXY;
		}
		else if (dest.y < OUR_GOAL_MINY )  {
			dest.y = OUR_GOAL_MINY;
		}
		if (botID == 2)
		{
			dest.y = -dest.y;
		}
		//print("RayCast")
		GoToPointStraight(botID, state, dest, PI / 2, false, false);
		save_goal(botID, state, Vector2D<float>(state->ballPos.x, state->ballPos.y));
	}
}