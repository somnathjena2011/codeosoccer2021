//	Warning! Do not modify this file
#include "stdafx.h"
#include "skills.h"

using namespace HAL;
using namespace std;
namespace MyStrategy
{
	/***************************GoToPoint with pathplanner***************************************************/
	void dodge(BeliefState *state, int botID, Vec2D targetPos, float finalAngle, bool shouldAlign) {
		Vector2D<float> relativeVelocity = state->homeVel[botID] - state->ballVel;
		double timeRequired = Vec2D::dist(state->ballPos, state->homePos[botID]) / (relativeVelocity.abs() * 5);
		Vector2D<float> predictedBallPos = Vector2D<float>(state->ballPos.x, state->ballPos.y) + timeRequired*state->ballVel;

		finalAngle = Vec2D::angle(targetPos, intV(predictedBallPos));
		GoToPoint(botID, state, intV(predictedBallPos), finalAngle, true, true);
	}
	void GoToPoint(int botID, BeliefState *state, Vector2D<int> dpoint, float finalslope, bool increaseSpeed, bool shouldAlign, bool isRandom)
	{

		if (shouldAlign)
			maingotopoint(botID, state, dpoint, 100, finalslope, CLEARANCE_PATH_PLANNER, increaseSpeed, true, isRandom);
		else
			maingotopoint(botID, state, dpoint, 100, finalslope, 0, increaseSpeed, true, isRandom);
	}
	void GoToPointStraight(int botID, BeliefState *state, Vector2D<int> dpoint, float finalslope, bool increaseSpeed, bool shouldAlign, bool isRandom)
	{
		if (shouldAlign)
			maingotopoint(botID, state, dpoint, 0, finalslope, CLEARANCE_PATH_PLANNER, increaseSpeed, false, isRandom);
		else
			maingotopoint(botID, state, dpoint, 0, finalslope, 0, increaseSpeed, false, isRandom);

	}
	void maingotopoint(int botID, BeliefState *state, Vector2D<int> dpoint, float finalvel, float finalslope, float clearance, bool increaseSpeed, bool avoid_obstacle, bool isRandom, bool isBallAnObstacle)
	{
		int prevVel = 0;
		static LocalAvoidance*    pathPlanner;
		pathPlanner = new LocalAvoidance();
		Comm* comm = Comm::getInstance();
		std::vector<obstacle> obs;
		obstacle obsTemp;
		for (int i = 0; i < HomeTeam::SIZE; ++i)
		{
			/// Adding Condition to REMOVE all obstacles that are sufficiently CLOSE to BALL
			if (i != botID && Vector2D<int>::distSq(state->homePos[botID], state->homePos[i]) < COLLISION_DIST * COLLISION_DIST && Vector2D<int>::distSq(state->ballPos, state->homePos[i]) > BOT_RADIUS * BOT_RADIUS * 9)
			{
				/// If destination is close to bot, don't add it as obstacle
				if (Vector2D<int>::distSq(dpoint, state->homePos[i]) > BOT_RADIUS * BOT_RADIUS * 1)
				{
					obsTemp.x = state->homePos[i].x;
					obsTemp.y = state->homePos[i].y;
					obsTemp.radius = BOT_RADIUS;
					obs.push_back(obsTemp);
				}
			}
		}

		for (int i = 0; i < AwayTeam::SIZE; ++i)
		{
			if (Vector2D<int>::distSq(state->homePos[botID], state->awayPos[i]) < COLLISION_DIST * COLLISION_DIST && Vector2D<int>::distSq(state->ballPos, state->awayPos[i]) > BOT_RADIUS * BOT_RADIUS * 9)
			{
				/// If destination is close to bot, don't add it as obstacle
				if (Vector2D<int>::distSq(dpoint, state->homePos[i]) > BOT_RADIUS * BOT_RADIUS * 1)
				{
					obsTemp.x = state->awayPos[i].x;
					obsTemp.y = state->awayPos[i].y;
					obsTemp.radius = BOT_RADIUS;
					obs.push_back(obsTemp);
				}
			}
		}
		Vector2D<int> point, nextWP, nextNWP;
		float r = 0, t = 0, dist = 0;
		dist = Vector2D<int>::dist(dpoint, state->homePos[botID]);  // Distance of next waypoint from the bot
		if (dist < BOT_POINT_THRESH)
		{
			float angle = fabs((float)firaNormalizeAngle(state->homeAngle[botID] - finalslope));
			if (angle > DRIBBLER_BALL_ANGLE_RANGE)
			{
				TurnToAngle(botID, state, finalslope);
				return;
			}

			comm->sendCommand(botID, finalvel, finalvel);
			return;
			// This part of the function is just for safety.
			// The tactic should actually prevent this call.
			// The bot should be aligned properly before this condition is reached.
		}

		pathPlanner->plan(state->homePos[botID],
			state->homeVel[botID],
			dpoint,
			obs,
			botID,
			true,
			state->homeAngle[botID],
			finalslope,
			t,
			r,
			comm,
			clearance,
			avoid_obstacle);

		float fTheta = asin(sqrt(fabs((float)r)));
		fTheta = 1 - fTheta / (PI / 2);
		fTheta = pow(fTheta, 2.2);
		float fDistance = (dist > BOT_POINT_THRESH * 3) ? 1 : dist / ((float)BOT_POINT_THRESH * 3);
		float fTot = fDistance * fTheta;
		if (isRandom)
			fTot = 0.5 + fTot*(1 - 0.3);
		else {
			if (botID == 0) {
				fTot = 0.2 + fTot*(1 - 0.2);
			}
			else {
				fTot = 0.5 + fTot*(1 - 0.5);
			}
		}

		float profileFactor = MAX_BOT_SPEED*fTot;

		if (botID == 0) {
			r *= 0.2*profileFactor * 2;
			t *= profileFactor*1.1;
		}
		else {
			r *= 0.2*profileFactor*1.7;
			t *= profileFactor*1.1;
		}

#if FIRA_COMM || FIRASSL_COMM
		comm->sendCommand(botID, (t - r), (t + r));
#else
		comm->sendCommand(botID, t - r, t + r);
#endif
	}
	/*******************************************************************************/



	/************************************TurnToAngle*********************************/
	void TurnToAngle(int botID, BeliefState *state, float angle)
	{
		Comm *comm = Comm::getInstance();
		float vl, vr;
		float finalSlope = angle;
		float turnAngleLeft = normalizeAngle(finalSlope - state->homeAngle[botID]); // Angle left to turn

		if (turnAngleLeft > PI / 2 || turnAngleLeft < -PI / 2)
		{
			if (turnAngleLeft > PI / 2)
				turnAngleLeft = turnAngleLeft - PI;
			else
				turnAngleLeft = turnAngleLeft + PI;
		}

		float factor = (turnAngleLeft + (turnAngleLeft)) / (PI / 2);
		vr = -0.4*MAX_BOT_OMEGA*(factor) / (PI / 2);
		vl = -vr;

		if (fabs((float)turnAngleLeft) > DRIBBLER_BALL_ANGLE_RANGE / 2)
		{
#if FIRA_COMM || FIRASSL_COMM 
			comm->sendCommand(botID, vl, vr);
#else
			comm->sendCommand(botID, vr, vl);
#endif
		}
		else
		{
			comm->sendCommand(botID, 0, 0);
		}
	}
	/*********************************************************************************/

	/*************************************GoToBall************************************/
	void GoToBall(int botID, BeliefState *state, bool align)
	{
		Comm *comm = Comm::getInstance();
		Vector2D<int> ballInitialpos, ballFinalpos;
		ballInitialpos = state->ballPos;
		ballFinalpos.x = state->ballPos.x + (0.7*state->ballVel.x);
		ballFinalpos.y = state->ballPos.y + (0.7*state->ballVel.y);

		float botballdist = Vector2D<int>::dist(ballInitialpos, state->homePos[botID]);
		float botballVeldist = Vector2D<int>::dist(ballFinalpos, state->homePos[botID]);

		Vec2D goalPoint(HALF_FIELD_MAXX, 0);
		float theta = Vector2D<int>::angle(ballInitialpos, goalPoint);

		if (align == true)
		{
			if ((Vector2D<int>::dist(ballInitialpos, state->homePos[botID])) >= BOT_BALL_THRESH)
			{
				if (fabs((float)tan(Vector2D<int>::angle(ballInitialpos, ballFinalpos)) - tan(Vector2D<int>::angle(state->homePos[botID], ballFinalpos))) < 1)
					maingotopoint(botID, state, ballInitialpos, 0, theta, CLEARANCE_PATH_PLANNER, false);
				else
					maingotopoint(botID, state, ballFinalpos, 0, theta, CLEARANCE_PATH_PLANNER, false);
			}
			else
			{
				comm->sendCommand(botID, 0, 0);
			}
		}
		else
		{
			if ((Vector2D<int>::dist(ballInitialpos, state->homePos[botID])) >= BOT_BALL_THRESH)
			{
				maingotopoint(botID, state, ballInitialpos, 0, theta, 00.0, false);
			}
			else
			{
				comm->sendCommand(botID, 0, 0);
			}
		}
	}
	/**********************************************************************************************/

	void Stop(int botID)
	{
		Comm *comm = Comm::getInstance();
		comm->sendCommand(botID, 0, 0);
	} // stop

	void Velocity(int botID, int vl, int vr)
	{
		Comm *comm = Comm::getInstance();
		comm->sendCommand(botID, vl, vr);
	}

	void Spin(int botID, float angularSpeed)// speed in radians
	{
		Comm *comm = Comm::getInstance();
		float omega, vl, vr;
		vr = (angularSpeed * (MAX_BOT_SPEED)) / MAX_BOT_OMEGA;
		vl = -vr;

#if FIRA_COMM || FIRASSL_COMM
		comm->sendCommand(botID, vl, vr);
#else
		comm->sendCommand(botID, vr, vl);
#endif
	}

	bool pointyInField(Vector2D<int> final)
	{
		if (final.y < -HALF_FIELD_MAXY + BALL_AT_CORNER_THRESH || final.y > HALF_FIELD_MAXY - BALL_AT_CORNER_THRESH)
			return false;
		else return true;
	}

	void save_goal(int botID, BeliefState *state, Vector2D<float> point) {
		int botx = state->homePos[botID].x;
		int boty = state->homePos[botID].y;

		int ballx = state->ballPos.x;
		int bally = state->ballPos.y;
		if (Vec2D::dist(state->homePos[botID], state->ballPos) < 340) {
			if (boty < bally) {
				Spin(botID, MAX_BOT_OMEGA);
			}
			else {
				Spin(botID, -MAX_BOT_OMEGA);

			}
			return;
		}
	}


	int rayCastY(BeliefState* state, int botID, bool isGK)
	{
		Vec2D ballpos = state->ballPos;
		Vector2D<float> ballVel = state->ballVel;
		//if (ballVel.x < 0)
		{
			float slope = ballVel.y / ballVel.x;
			float y = (state->homePos[botID].x - ballpos.x) * slope + ballpos.y;
			return y;
		}
		//return 0;
	}

	strips whichStrip(int x, int y)
	{
		if (y > 3500 / 3)
		{
			return BOTTOM_STRIP;
		}
		else if (y < -3500 / 3)
		{
			return TOP_STRIP;
		}
		return MIDDLE_STRIP;
	}

	// Use only when distance between bot and ball is small
	Vec2D predictBallPos(BeliefState* state, int botID)
	{
		float botVel = 120;
		Vec2D ballPoint(state->ballPos.x, state->ballPos.y);
		Vector2D<float> ballVel = state->ballVel;
		float dist = Vec2D::dist(state->homePos[botID], ballPoint);
		float time = dist / botVel;
		Vec2D predictedPos;
		predictedPos.x = ballPoint.x + ballVel.x * time;
		predictedPos.y = ballPoint.y + ballVel.y * time;

		return predictedPos;

	}

	// For attacker to put ball in Dbox
	void shootForAssist(BeliefState *state, int botID)
	{
		int botx = state->homePos[botID].x;
		int boty = state->homePos[botID].y;

		int ballx = state->ballPos.x;
		int bally = state->ballPos.y;
		if (Vec2D::dist(state->homePos[botID], state->ballPos) < 340) {
			if (boty < bally) {
				Spin(botID, -MAX_BOT_OMEGA / 2);
			}
			else {
				Spin(botID, +MAX_BOT_OMEGA / 2);

			}
			return;
		}
		Vec2D dpoint = predictBallPos(state, botID);
		GoToPoint(botID, state, dpoint, PI / 2, true, true);
	}

	void shootForGoal(BeliefState * state, int botID) {
		int botx = state->homePos[botID].x;
		int boty = state->homePos[botID].y;

		int ballx = state->ballPos.x;
		int bally = state->ballPos.y;
		if (Vec2D::dist(state->homePos[botID], state->ballPos) < 340) {
			if (abs(boty - OUR_GOAL_MINY) < abs(boty - OUR_GOAL_MAXY)) {
				Spin(botID, -MAX_BOT_OMEGA/2);
			}
			else {
				Spin(botID, MAX_BOT_OMEGA/2);

			}
			return;
		}
		Vec2D dpoint = predictBallPos(state, botID);
		//GoToPointStraight(botID, state, dpoint, PI / 2, true, true);
	}

	Vector2D<float> floatV(Vec2D v) {
		return Vector2D<float>(v.x, v.y);
	}

	Vector2D<int> intV(Vector2D<float> v) {
		return Vector2D<int>(v.x, v.y);
	}
	void vibrate(BeliefState *state, int botID, int c)
	{

		Vector2D<int> s1, s2;
		s1.x = state->homePos[botID].x;
		s1.y = c + HALF_FIELD_MAXY*.3;
		s2.x = state->homePos[botID].x;
		s2.y = c - HALF_FIELD_MAXY*.3;

		if (abs(state->homePos[botID].y - s1.y) <= BOT_BALL_THRESH)
		{
			Velocity(botID, 0, 0);
			GoToPoint(botID, state, s2, 90, false, false);

		}
		else if (abs(state->homePos[botID].y - s2.y) <= BOT_BALL_THRESH)
		{
			Velocity(botID, 0, 0);
			GoToPoint(botID, state, s1, 90, false, false);

		}
		if (state->homeVel[botID].y > 0)
			GoToPoint(botID, state, s1, 90, false, true);
		else if (state->homeVel[botID].y < 0)
			GoToPoint(botID, state, s2, 90, false, true);


		//	GoToPoint(botID,state,s2, 90,false, false);
	//	else if(abs(state->homePos[botID].y-s2.y)==BOT_BALL_THRESH)
		//	GoToPoint(botID,state,s1, 90,false, false);


	}

} // namespace MyStrategy
