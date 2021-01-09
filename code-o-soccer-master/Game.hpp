// For adding header files define your includes here and add the headers in Game folder.
// For example, You may see these files - Attacker.hpp,Defender.hpp and Goalkeeper.hpp
// For checking out the skills you may see skills.h placed in Skills folder.
#pragma once
#include "skills.h"
#include "Attacker.hpp"
#include "Striker.hpp"
#include "Defender.hpp"
#include "GoalKeeper.hpp"

// Change your team color here (BLUE_TEAM/YELLOW_TEAM)face

Simulator::TeamColor teamColor = Simulator::BLUE_TEAM;

// Make usingDebugger is false when playing against an opponent
bool usingDebugger = true;

namespace MyStrategy
{
	extern int defend_bot = 1;
	extern int attack_bot = 2;

	// Write your strategy here in game function.
	// You can also make new functions and call them from game function.
	void game(BeliefState *state)
	{
		//attacker(state, 2);
		//defender(state,1);

		///*Defender modes*/
		if (state->ballPos.x > 0) {
			strips currStrip = whichStrip(state->ballPos.x, state->ballPos.y);
			// Shoot mode
			if (abs(state->ballPos.y) < OUR_GOAL_MAXY) {
				striker(state, 1, MIDDLE_STRIP, true);
			}
			// Wait mode
			else
			{
				striker(state, 1, currStrip);
			}
		}
		else {
			defender(state, 1);
		}

		/*Attcker modes*/
		if (state->ballPos.x < -HALF_FIELD_MAXX + 2.5 * DBOX_WIDTH)
		{
			goalkeeper(state, 2);
		}
		else
		{
			attacker(state, 2);
		}
		goalkeeper(state, 0);
		//print("%d %d", state->ballPos.x, state->ballPos.y);
	}
}