
#include "stdafx.h"

#include "CStreamListener.h"

void StreamListener::on_frame_ready(astra::StreamReader& reader,
									astra::Frame& frame)
{
	//std::cout << "Frame ready!" << std::endl;

	auto bodyFrame = frame.get<astra::BodyFrame>();

	if (bodyFrame.is_valid())
	{
		auto bodies = bodyFrame.bodies();



		/*for (auto body : bodies)
		{
			auto joint1 = body.joints();
			auto joint = joint1[0];

			auto x = joint.depth_position();
			std::cout << x.x << std::endl;
		}*/
	}

	auto handFrame = frame.get<astra::HandFrame>();
	if (handFrame.is_valid())
	{
		auto m_handPoints = handFrame.handpoints();
	}
}
