#pragma once

#define BIT(x) (1<<(x))
enum collisiontypes {
	COL_NOTHING = 0, //<Collide with nothing
	COL_MANDIBLE = BIT(0), //<Collide with ships
	COL_SKULL = BIT(1), //<Collide with walls
	COL_DISC = BIT(2) //<Collide with powerups
};

int mandibleCollidesWith = COL_SKULL | COL_DISC;
int skullCollidesWith = COL_MANDIBLE | COL_DISC;
int ligamentCollidesWith = COL_NOTHING;
int muscleCollidesWith = COL_NOTHING;
int discCollidesWith = COL_SKULL | COL_MANDIBLE;