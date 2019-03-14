/**
* @file CAstraStream.h
* @brief Currently only used as frame listener
*
*
*/


#pragma once

#include <astra\astra.hpp>

#include "CStreamListener.h"

#define LICENSE_KEY "Okk/ujeQpYTstUwg8FPWhWztPS/nR31ZNAOMioFZMXpPegCjscSpAse7zvsEOoL9H1CafIzWEd++wyiyzU5jCk5hbWU9S3Jpc3RpamFuIExlbmFjfE9yZz1Vbml2ZXJzaXR5IG9mIFJpamVrYXxDb21tZW50PXxFeHBpcmF0aW9uPTk5OTk5OTk5OTk="

/// Class AstraStream which is used as FrameListener (for now)
// TODO: implement objected oriented design with objective OpenGL
class AstraStream : public astra::FrameListener
{
public:
	/// Control variable which determines whether stream is running
	bool										streamRunning;

	// Constructor
												AstraStream();
	// Initializes camera
	// @returns true if init is successfull
	// @returns false if init is not successfull
	//bool										init();
	// Properly terminates the camera
	// @returns nothing
	//void										terminate();
	/// FrameListener which acts as interrup when frame is recieved
	/// @returns nothing
	virtual	void								on_frame_ready(
														astra::StreamReader& reader,
														astra::Frame& frame) override;
	// Destructor
												~AstraStream();
	/// Getter for currently tracked bodies
	/// @returns currently tracked bodies or NULL
	astra::BodyList								getBodies();

	/// Getter for currently tracked body masks
	/// @returns currently tracked body mask or NULL
	astra::BodyMask								getBodyMask();
	// Getter for currently tracked hands
	// @returns currently tracked hand points or NULL
	//astra::HandFrame::HandPointList				getHandPoints();

	const astra::Vector3f*							m_nearestPoint;
	astra::PointFrame								m_pointFrame;
private:
	// Basic astra stream variable needed to start a stream
	//astra::StreamSet							m_sensor;
	// Basic astra stream variable which can get frames of started stream
	//astra::StreamReader							m_reader;
	// Stream listener which extends astra::FrameListener
	//StreamListener							m_listener;
	/// Currently tracked bodies
	astra::BodyList								m_bodies;
	///Currently tracked body masks
	astra::BodyMask								m_bodyMask;
	// Currently tracked hand points
	//astra::HandFrame::HandPointList				m_handPoints;



protected:

	//astra::DepthStream						configure_depth(astra::StreamReader& reader);
};
