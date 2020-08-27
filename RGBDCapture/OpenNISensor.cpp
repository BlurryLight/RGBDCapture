#include "OpenNISensor.h"
#include <chrono>
#include <iostream>


OpenNISensor::OpenNISensor() {
  m_flagInitSuccessful = m_flagShowImage = true;
  m_frameNum = m_frameIdx = 0;
  m_sensorType = 0;
  auto time_point = std::chrono::system_clock::now();
  auto duration = time_point.time_since_epoch();
  long seconds_from_epoch =
      std::chrono::duration_cast<std::chrono::seconds>(duration).count();
  m_prefix = std::to_string(seconds_from_epoch);

  init();
#ifndef SAVEJPG
  this->compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
  this->compression_params.push_back(9); // max level compression
#else
  this->compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
  this->compression_params.push_back(85);
#endif
}

OpenNISensor::~OpenNISensor() {
  m_depthStream.stop();
  m_colorStream.stop();
  m_depthStream.destroy();
  m_colorStream.destroy();
  m_device.close();
  openni::OpenNI::shutdown();
}

bool OpenNISensor::init() {
  openni::Status rc = openni::STATUS_OK;
  rc = openni::OpenNI::initialize();
  if (rc != openni::STATUS_OK) {
    std::cerr << "OpenNI Initial Error: " << openni::OpenNI::getExtendedError()
              << std::endl;
    openni::OpenNI::shutdown();
    m_flagInitSuccessful = false;
    return m_flagInitSuccessful;
  }

  const char *deviceURI = openni::ANY_DEVICE;
  rc = m_device.open(deviceURI);
  if (rc != openni::STATUS_OK) {
    cerr << "ERROR: Can't Open Device: " << openni::OpenNI::getExtendedError()
         << endl;
    openni::OpenNI::shutdown();
    m_flagInitSuccessful = false;
    return m_flagInitSuccessful;
  }

  rc = m_depthStream.create(m_device, openni::SENSOR_DEPTH);

  if (rc == openni::STATUS_OK) {
    rc = m_depthStream.start();

    if (rc != openni::STATUS_OK) {
      cerr << "ERROR: Can't start depth stream on device: "
           << openni::OpenNI::getExtendedError() << endl;
      m_depthStream.destroy();
      m_flagInitSuccessful = false;
      return m_flagInitSuccessful;
    }
  } else {
    cerr << "ERROR: This device does not have depth sensor" << endl;
    openni::OpenNI::shutdown();
    m_flagInitSuccessful = false;
    return m_flagInitSuccessful;
  }

  rc = m_colorStream.create(m_device, openni::SENSOR_COLOR);
  if (rc == openni::STATUS_OK) {
    rc = m_colorStream.start();
    if (rc != openni::STATUS_OK) {
      cerr << "ERROR: Can't start color stream on device: "
           << openni::OpenNI::getExtendedError() << endl;
      m_colorStream.destroy();
      m_flagInitSuccessful = false;
      return m_flagInitSuccessful;
    }
  } else {
    cerr << "ERROR: This device does not have color sensor" << endl;
    openni::OpenNI::shutdown();
    m_flagInitSuccessful = false;
    return m_flagInitSuccessful;
  }


  if (!m_depthStream.isValid() || !m_colorStream.isValid()) {
    cerr << "SimpleViewer: No valid streams. Exiting" << endl;
    m_flagInitSuccessful = false;
    openni::OpenNI::shutdown();
    return m_flagInitSuccessful;
  }

  openni::VideoMode depthVideoMode = m_depthStream.getVideoMode();
  openni::VideoMode colorVideoMode = m_colorStream.getVideoMode();
  m_depthWidth = depthVideoMode.getResolutionX();
  m_depthHeight = depthVideoMode.getResolutionY();
  m_colorWidth = colorVideoMode.getResolutionX();
  m_colorHeight = colorVideoMode.getResolutionY();
  cout << "Depth = (" << m_depthWidth << "," << m_depthHeight << ")" << endl;
  cout << "Color = (" << m_colorWidth << "," << m_colorHeight << ")" << endl;

  // Set exposure if needed
  m_colorStream.getCameraSettings()->setAutoWhiteBalanceEnabled(false);
  int exposure = m_colorStream.getCameraSettings()->getExposure();
  int delta = 100;
  m_colorStream.getCameraSettings()->setExposure(exposure + delta);
  m_flagInitSuccessful = true;
#ifdef REGISTRATION
  if (m_device.isImageRegistrationModeSupported(
          openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR)) {
    m_device.setImageRegistrationMode(
        openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
    std::cout << "registration is ok" << std::endl;
  }
#endif
  return m_flagInitSuccessful;
}

static bool scan_flag = false;
static bool continous_flag = false;
void OpenNISensor::scan() {
  if (!m_flagInitSuccessful) {
    cout << "WARNING: initialize the device at first!" << endl;
    return;
  }

  createRGBDFolders();

  string strDepthWindowName("Depth"), strColorWindowName("Color");

#ifndef CV_NOSHOW
  cv::namedWindow(strDepthWindowName, cv::WINDOW_AUTOSIZE);
  cv::namedWindow(strColorWindowName, cv::WINDOW_AUTOSIZE);
#endif
  auto get_a_frame = [&]() -> void {
    m_colorStream.readFrame(&m_colorFrame);
    m_depthStream.readFrame(&m_depthFrame);
    auto timestamp = m_colorFrame.getTimestamp();
#ifndef SAVETIMESTAMP
    auto imgIndex = m_frameIdx;
#else
    auto imgIndex = timestamp;
#endif
    if (m_colorFrame.isValid()) {
      cv::Mat mImageRGB(m_colorHeight, m_colorWidth, CV_8UC3,
                        (void *)m_colorFrame.getData());
      cv::Mat cImageBGR;
      cv::cvtColor(mImageRGB, cImageBGR, cv::COLOR_RGB2BGR);
      if (m_sensorType == 0)
        cv::flip(cImageBGR, cImageBGR, 1);
#ifndef CV_NOSHOW
      cv::imshow(strColorWindowName, cImageBGR);
#endif

#ifndef SAVEJPG
      std::string suffix{".png"};
#else
      std::string suffix{".jpg"};
#endif

      cv::imwrite(m_strRGBDFolder + "/rgb/" + m_prefix + "_" +
                      to_string(imgIndex) + suffix,
                  cImageBGR, this->compression_params);
    } else {
      cerr << "ERROR: Cannot read color frame from color stream. Quitting..."
           << endl;
      return;
    }

    if (m_depthFrame.isValid()) {
      cv::Mat mImageDepth(m_depthHeight, m_depthWidth, CV_16UC1,
                          (void *)m_depthFrame.getData());
      cv::Mat cScaledDepth;
      mImageDepth.convertTo(cScaledDepth, CV_16UC1, kDepthScaleFactor);
      if (m_sensorType == 0)
        cv::flip(cScaledDepth, cScaledDepth, 1);
      cv::Mat cNormedDepth;
      cv::normalize(cScaledDepth, cNormedDepth, 0, 255, NORM_MINMAX);
#ifndef CV_NOSHOW
      cv::Mat cColoredDepth;
      cNormedDepth.convertTo(cNormedDepth,CV_8UC1); //coplor map only support 8UC1
      cv::applyColorMap(cNormedDepth, cColoredDepth, COLORMAP_JET);
      cv::imshow(strDepthWindowName, cColoredDepth);
#endif

      cv::imwrite(m_strRGBDFolder + "/depth/" + m_prefix + "_" +
                      to_string(imgIndex) + ".png",
                  cScaledDepth);
    } else {
      cerr << "ERROR: Cannot read depth frame from depth stream. Quitting..."
           << endl;
      return;
    }
    m_frameIdx++;
  };
  while (true) {
#ifndef CV_NOSHOW
  char key = cv::waitKey(1);
  if (key == 27) {
    break;
  } else if (key == 32) {
    continous_flag = !continous_flag;
    std::cout << (continous_flag ? "continuous capture" : "press capture")
              << std::endl;
  } else if (!continous_flag && (key == 13 || key == 110)) // enter or n
  {
    std::cout << this->m_frameIdx << std::endl;
    get_a_frame();
  }

  if (continous_flag) {
    std::cout << this->m_frameIdx << std::endl;
    get_a_frame();
  }
#else
  std::cout << this->m_frameIdx << std::endl;
  get_a_frame();
#endif
}
}
