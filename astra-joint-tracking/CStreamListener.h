/**
* @file CStreamListener.h
* @brief Currently unused
*
*
*/

#pragma once

//#include <astra\astra.hpp>
#include "CAstraStream.h"

/// Class currently not used, only extends astra::Frame listener to act as a interrupt when frame is ready
class StreamListener : public astra::FrameListener
{
public:
	/// FrameListener which acts as interrup when frame is recieved
	/// @returns nothing
	virtual void on_frame_ready(astra::StreamReader& reader,
								astra::Frame& frame) override;

private:
	
};
