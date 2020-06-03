/**
 * Phase.h
 * Purpose: Collection of Phase Enums
 * @author Mintae Kim
 */

#pragma once

enum class BrickPhase
{
	INIT,
	ENABLE,
	UNABLE,
	SELECTED,
	GRABBED,
	LIFTED,
	RELEASED,
	DONE
};

enum class RobotPhase
{
	STOP,
	MOVING, // -> Src
	GRAB,
	LIFTING, // -> Dst
	RELEASE,
};

enum class ClientPhase
{	
	DISCONNECTED,
	ACCEPTED,
	CONNECTED,
};