#ifndef OPENNISENSOR_H
#define OPENNISENSOR_H

#include "OpenNI.h"
#include "RGBDSensor.h"
#include "global.h"
#include <opencv2/opencv.hpp>
#include <string>

class OpenNISensor : public RGBDSensor
{
public:
  OpenNISensor();

  ~OpenNISensor();

  bool init();

  void scan();

private:
	openni::Device			m_device;
	openni::VideoStream		m_depthStream;
	openni::VideoStream		m_colorStream;
	openni::VideoFrameRef	m_depthFrame;
	openni::VideoFrameRef	m_colorFrame;
	bool m_flagInitSuccessful;
	bool m_flagShowImage;
        std::string m_prefix;
};


#endif
