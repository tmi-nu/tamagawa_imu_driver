/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef TAMAGAWA_IMU_DRIVER_TAG_SERIAL_DRIVER_H_
#define TAMAGAWA_IMU_DRIVER_TAG_SERIAL_DRIVER_H_

/**
 * @file tag_serial_driver.h
 * @brief Tamagawa IMU serial driver class
 */

#include <string>

#include <boost/asio.hpp>

#include <diagnostic_updater/diagnostic_updater.h>
#include <sensor_msgs/Imu.h>

namespace as = boost::asio;

class TagSerialDriver
{
public:
  /**
   * @brief constructor
   */
  TagSerialDriver();

  /**
   * @brief initialize and open serial port.
   * @return true on success or false on failure
   */
  bool initialize();

  /**
   * @brief close serial port.
   */
  void finalize();

private:
  using DiagStatus = diagnostic_msgs::DiagnosticStatus;

  /**
   * @brief check IMU status
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   */
  void checkStatus(diagnostic_updater::DiagnosticStatusWrapper & stat);

  /**
   * @brief Handler to be called when the read operation completes.
   * @param [in] error Error argument of a handler
   * @param [in] bytes_transfered Bytes transferred argument of a handler
   */
  void onRead(const boost::system::error_code & error, const std::size_t bytes_transfered);

  /**
   * @brief Handler to be called when the write operation completes.
   * @param [in] error Error argument of a handler
   * @param [in] bytes_transfered Bytes transferred argument of a handler
   */
  void onWrite(const boost::system::error_code & error, std::size_t bytes_transfered);

  /**
   * @brief timer callback
   * @param [in] event timing information
   */
  void onTimer(const ros::TimerEvent & event);

  /**
 * @brief verify checksum
 * @param [in] data start of message
 * @return true on success or false on failure
 */
  bool verifyChecksum(const std::string & data);

  ros::NodeHandle nh_{""};    //!< @brief ros node handle
  ros::NodeHandle pnh_{"~"};  //!< @brief private ros node handle

  ros::Publisher pub_;        //!< @brief publisher for IMU data
  sensor_msgs::Imu imu_msg_;  //!< @brief IMU message
  std::string imu_frame_id_;  //!< @brief frame ID for sensor data
  std::string port_;          //!< @brief serial port name
  as::io_service io_;         //!< @brief facilities of custom asynchronous services
  boost::shared_ptr<as::serial_port> serial_port_;  //!< @brief wrapper over serial port
  as::streambuf buffer_;                            //!< @brief Received data
  bool is_checksum_ok_;                  //!< @brief flag if checksum error is occuring or not
  uint64_t receive_count_;               //!< @brief counter of received data
  uint16_t imu_status_;                  //!< @brief  IMU status
  ros::Timer timer_;                     //!< @brief timer
  diagnostic_updater::Updater updater_;  //!< @brief updater class which advertises to /diagnostics
};

#endif  // TAMAGAWA_IMU_DRIVER_TAG_SERIAL_DRIVER_H_
