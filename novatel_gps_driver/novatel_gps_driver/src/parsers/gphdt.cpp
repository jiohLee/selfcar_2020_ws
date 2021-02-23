// *****************************************************************************
//
// Copyright (c) 2017, Southwest Research Institute® (SwRI®)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL SOUTHWEST RESEARCH INSTITUTE BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#include <novatel_gps_driver/parsers/gphdt.h>
#include <boost/make_shared.hpp>

#include <swri_string_util/string_util.h>

const std::string novatel_gps_driver::GphdtParser::MESSAGE_NAME = "GPHDT";

uint32_t novatel_gps_driver::GphdtParser::GetMessageId() const
{
  return 0;
}

const std::string novatel_gps_driver::GphdtParser::GetMessageName() const
{
  return MESSAGE_NAME;
}

novatel_gps_msgs::GphdtPtr novatel_gps_driver::GphdtParser::ParseAscii(const novatel_gps_driver::NmeaSentence& sentence) throw(ParseException)
{
  // Check the length first -- should be 15 elements long
  const size_t length = 3;

  if (sentence.body.size() > length)
  {
    std::stringstream error;
    error << "Expected GPGGA length : " << length << ", actual length = " << sentence.body.size();
    throw ParseException(error.str());
  }

  novatel_gps_msgs::GphdtPtr msg = boost::make_shared<novatel_gps_msgs::Gphdt>();

  msg->message_id = sentence.body[0];


  bool valid = true;

  double heading = 0.0;
  valid = valid && ParseDouble(sentence.body[1], heading);
  std::cout << "heading : " << heading << std::endl;

  msg->heading = heading;

  if (!valid)
  {
    throw ParseException("GPHDT log was invalid.");
  }

//  // Check for actual lat and lon data
  if (sentence.body[1].empty())
  {
    // No heading data, return false;
    was_last_gps_valid_ = false;
  }

  was_last_gps_valid_ = true;

  return msg;
}

bool novatel_gps_driver::GphdtParser::WasLastGpsValid() const
{
  return was_last_gps_valid_;
}
