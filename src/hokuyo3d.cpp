/*
 * Copyright (c) 2014, ATR, Atsushi Watanabe
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <boost/bind.hpp>
#include <boost/chrono.hpp>
#include <boost/thread.hpp>
#include <boost/thread/lock_guard.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/asio.hpp>

#include <algorithm>
#include <deque>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <vssp.h>

class Hokuyo3dNode
{
public:
  void cbPoint(
      const vssp::Header& header,
      const vssp::RangeHeader& range_header,
      const vssp::RangeIndex& range_index,
      const boost::shared_array<uint16_t>& index,
      const boost::shared_array<vssp::XYZI>& points,
      const boost::posix_time::ptime& time_read)
  {
    if (timestamp_base_ == ros::Time(0)) return;
    // Pack scan data
    if (enable_pc_)
    {
      if (cloud_.points.size() == 0)
      {
        // Start packing PointCloud message
        cloud_.header.frame_id = frame_id_;
        cloud_.header.stamp = timestamp_base_ + ros::Duration(range_header.line_head_timestamp_ms * 0.001);
      }
      // Pack PointCloud message
      for (int i = 0; i < index[range_index.nspots]; i++)
      {
        if (points[i].r < range_min_)
        {
          continue;
        }
        geometry_msgs::Point32 point;
        point.x = points[i].x;
        point.y = points[i].y;
        point.z = points[i].z;
        cloud_.points.push_back(point);
        cloud_.channels[0].values.push_back(points[i].i);
      }
    }
    if (enable_pc2_)
    {
      if (cloud2_frame_.data.size() == 0)
      {
        // Start packing PointCloud2 message
        cloud2_frame_.header.frame_id = frame_id_;
        cloud2_frame_.header.stamp = timestamp_base_ + ros::Duration(range_header.line_head_timestamp_ms * 0.001);
        cloud2_frame_.row_step = 0;
        cloud2_frame_.width = 0;
      }
      // Pack PointCloud2 message
      cloud2_frame_.data.resize((cloud2_frame_.width + index[range_index.nspots]) * cloud2_frame_.point_step);

      float* data_frame = reinterpret_cast<float*>(&cloud2_frame_.data[0]);
      data_frame += cloud2_frame_.width * cloud2_frame_.point_step / sizeof(float);

      if (cloud2_field_.data.size() == 0)
      {
        // Start packing PointCloud2 message
        cloud2_field_.header.frame_id = frame_id_;
        cloud2_field_.header.stamp = timestamp_base_ + ros::Duration(range_header.line_head_timestamp_ms * 0.001);
        cloud2_field_.row_step = 0;
        cloud2_field_.width = 0;
      }
      // Pack PointCloud2 message
      cloud2_field_.data.resize((cloud2_field_.width + index[range_index.nspots]) * cloud2_field_.point_step);

      float* data_field = reinterpret_cast<float*>(&cloud2_field_.data[0]);
      data_field += cloud2_field_.width * cloud2_field_.point_step / sizeof(float);

      if (cloud2_line_.data.size() == 0)
      {
        // Start packing PointCloud2 message
        cloud2_line_.header.frame_id = frame_id_;
        cloud2_line_.header.stamp = timestamp_base_ + ros::Duration(range_header.line_head_timestamp_ms * 0.001);
        cloud2_line_.row_step = 0;
        cloud2_line_.width = 0;
      }
      // Pack PointCloud2 message
      cloud2_line_.data.resize((cloud2_line_.width + index[range_index.nspots]) * cloud2_line_.point_step);

      float* data_line = reinterpret_cast<float*>(&cloud2_line_.data[0]);
      data_line += cloud2_line_.width * cloud2_line_.point_step / sizeof(float);

      for (int i = 0; i < index[range_index.nspots]; i++)
      {
        if (points[i].r < range_min_)
        {
          continue;
        }
        *(data_frame++) = points[i].x;
        *(data_frame++) = points[i].y;
        *(data_frame++) = points[i].z;
        *(data_frame++) = points[i].i;
        cloud2_frame_.width++;

        *(data_field++) = points[i].x;
        *(data_field++) = points[i].y;
        *(data_field++) = points[i].z;
        *(data_field++) = points[i].i;
        cloud2_field_.width++;

        *(data_line++) = points[i].x;
        *(data_line++) = points[i].y;
        *(data_line++) = points[i].z;
        *(data_line++) = points[i].i;
        cloud2_line_.width++;
      }
      cloud2_frame_.row_step = cloud2_frame_.width * cloud2_frame_.point_step;
      cloud2_field_.row_step = cloud2_field_.width * cloud2_field_.point_step;
      cloud2_line_.row_step = cloud2_line_.width * cloud2_line_.point_step;
    }
    
    // Publish points
    {
      if (enable_pc_)
      {
        if (cloud_.header.stamp < cloud_stamp_last_ && !allow_jump_back_)
        {
          ROS_INFO("Dropping timestamp jump backed cloud");
        }
        else
        {
          pub_pc_.publish(cloud_);
        }
        cloud_stamp_last_ = cloud_.header.stamp;
        cloud_.points.clear();
        cloud_.channels[0].values.clear();
      }
      if (enable_pc2_)
      {
        if(range_header.frame != frame_)
        {
          cloud2_frame_.data.resize(cloud2_frame_.width * cloud2_frame_.point_step);
          if (cloud2_frame_.header.stamp < cloud2_frame_stamp_last_ && !allow_jump_back_)
          {
            ROS_INFO("Dropping timestamp jump backed cloud2_fame_");
          }
          else
          {
            pub_pc2_frame_.publish(cloud2_frame_);
          }
          cloud2_frame_stamp_last_ = cloud2_frame_.header.stamp;
          cloud2_frame_.data.clear();
        }

        if((range_header.field != field_ || range_header.frame != frame_))
        {
          cloud2_field_.data.resize(cloud2_field_.width * cloud2_field_.point_step);
          if (cloud2_field_.header.stamp < cloud2_field_stamp_last_ && !allow_jump_back_)
          {
            ROS_INFO("Dropping timestamp jump backed cloud2_field");
          }
          else
          {
            pub_pc2_field_.publish(cloud2_field_);
          }
          cloud2_field_stamp_last_ = cloud2_field_.header.stamp;
          cloud2_field_.data.clear();
        }

        cloud2_line_.data.resize(cloud2_line_.width * cloud2_line_.point_step);
        if (cloud2_line_.header.stamp < cloud2_line_stamp_last_ && !allow_jump_back_)
        {
          ROS_INFO("Dropping timestamp jump backed cloud2");
        }
        else
        {
          pub_pc2_line_.publish(cloud2_line_);
        }
        cloud2_line_stamp_last_ = cloud2_line_.header.stamp;
        cloud2_line_.data.clear();
      }
      if (range_header.frame != frame_) ping();
      frame_ = range_header.frame;
      field_ = range_header.field;
      line_ = range_header.line;
    }

    int current_horizontal_interlace_ = 0, current_vertical_interlace_=0;
    pnh_.getParam("horizontal_interlace", current_horizontal_interlace_);
    pnh_.getParam("vertical_interlace", current_vertical_interlace_);
    if(current_horizontal_interlace_ != horizontal_interlace_)
    {
      if (current_horizontal_interlace_ <= 0 || current_horizontal_interlace_ > 20)
      {
        ROS_ERROR("Invalid horizontal_interlace value %d", current_horizontal_interlace_);
        pnh_.setParam("horizontal_interlace", horizontal_interlace_);
      }else
      {
        ROS_INFO("Setting horizontal_interlace param %d to %d", horizontal_interlace_, current_horizontal_interlace_);
        horizontal_interlace_ = current_horizontal_interlace_;
        pnh_.setParam("horizontal_interlace", horizontal_interlace_);
        if (set_auto_reset_) driver_.setAutoReset(auto_reset_);
        driver_.setHorizontalInterlace(horizontal_interlace_);
      }
    }
    if(current_vertical_interlace_ != vertical_interlace_)
    {
      if (current_vertical_interlace_ <= 0 || current_vertical_interlace_ > 10)
      {
        ROS_ERROR("Invalid vertical interlace value %d", current_vertical_interlace_);
        pnh_.setParam("vertical_interlace", vertical_interlace_);
      }else
      {
        ROS_INFO("Setting vertical_interlace param: %d to %d", vertical_interlace_, current_vertical_interlace_);
        vertical_interlace_ = current_vertical_interlace_;
        pnh_.setParam("vertical_interlace", vertical_interlace_);
        if (set_auto_reset_) driver_.setAutoReset(auto_reset_);
        driver_.setVerticalInterlace(vertical_interlace_);
        driver_.requestVerticalTable(vertical_interlace_);
      }
    }
  }

  void cbError(
      const vssp::Header& header,
      const std::string& message,
      const boost::posix_time::ptime& time_read)
  {
    ROS_ERROR("%s", message.c_str());
  }

  void cbPing(
      const vssp::Header& header,
      const boost::posix_time::ptime& time_read)
  {
    const ros::Time now = ros::Time::fromBoost(time_read);
    const ros::Duration delay =
        ((now - time_ping_) - ros::Duration(header.send_time_ms * 0.001 - header.received_time_ms * 0.001)) * 0.5;
    const ros::Time base = time_ping_ + delay - ros::Duration(header.received_time_ms * 0.001);

    timestamp_base_buffer_.push_back(base);
    if (timestamp_base_buffer_.size() > 5)
      timestamp_base_buffer_.pop_front();

    auto sorted_timstamp_base = timestamp_base_buffer_;
    std::sort(sorted_timstamp_base.begin(), sorted_timstamp_base.end());

    if (timestamp_base_ == ros::Time(0))
      timestamp_base_ = sorted_timstamp_base[sorted_timstamp_base.size() / 2];
    else
      timestamp_base_ += (sorted_timstamp_base[sorted_timstamp_base.size() / 2] - timestamp_base_) * 0.1;

    ROS_DEBUG("timestamp_base: %lf", timestamp_base_.toSec());
  }

  void cbAux(
      const vssp::Header& header,
      const vssp::AuxHeader& aux_header,
      const boost::shared_array<vssp::Aux>& auxs,
      const boost::posix_time::ptime& time_read)
  {
    if (timestamp_base_ == ros::Time(0))
      return;
    ros::Time stamp = timestamp_base_ + ros::Duration(aux_header.timestamp_ms * 0.001);

    if ((aux_header.data_bitfield & (vssp::AX_MASK_ANGVEL | vssp::AX_MASK_LINACC)) ==
        (vssp::AX_MASK_ANGVEL | vssp::AX_MASK_LINACC))
    {
      imu_.header.frame_id = imu_frame_id_;
      imu_.header.stamp = stamp;
      for (int i = 0; i < aux_header.data_count; i++)
      {
        imu_.orientation_covariance[0] = -1.0;
        imu_.angular_velocity.x = auxs[i].ang_vel.x;
        imu_.angular_velocity.y = auxs[i].ang_vel.y;
        imu_.angular_velocity.z = auxs[i].ang_vel.z;
        imu_.linear_acceleration.x = auxs[i].lin_acc.x;
        imu_.linear_acceleration.y = auxs[i].lin_acc.y;
        imu_.linear_acceleration.z = auxs[i].lin_acc.z;
        if (imu_stamp_last_ > imu_.header.stamp && !allow_jump_back_)
        {
          ROS_INFO("Dropping timestamp jump backed imu");
        }
        else
        {
          pub_imu_.publish(imu_);
        }
        imu_stamp_last_ = imu_.header.stamp;
        imu_.header.stamp += ros::Duration(aux_header.data_ms * 0.001);
      }
    }
    if ((aux_header.data_bitfield & vssp::AX_MASK_MAG) == vssp::AX_MASK_MAG)
    {
      mag_.header.frame_id = mag_frame_id_;
      mag_.header.stamp = stamp;
      for (int i = 0; i < aux_header.data_count; i++)
      {
        mag_.magnetic_field.x = auxs[i].mag.x;
        mag_.magnetic_field.y = auxs[i].mag.y;
        mag_.magnetic_field.z = auxs[i].mag.z;
        if (mag_stamp_last_ > imu_.header.stamp && !allow_jump_back_)
        {
          ROS_INFO("Dropping timestamp jump backed mag");
        }
        else
        {
          pub_mag_.publish(mag_);
        }
        mag_stamp_last_ = imu_.header.stamp;
        mag_.header.stamp += ros::Duration(aux_header.data_ms * 0.001);
      }
    }
  }

  void cbConnect(bool success)
  {
    if (success)
    {
      ROS_INFO("Connection established");
      ping();
      if (set_auto_reset_) driver_.setAutoReset(auto_reset_);
      driver_.setHorizontalInterlace(horizontal_interlace_);
      driver_.requestHorizontalTable();
      driver_.setVerticalInterlace(vertical_interlace_);
      driver_.requestVerticalTable(vertical_interlace_);
      driver_.requestData(true, true);
      driver_.requestAuxData();
      driver_.receivePackets();
      ROS_INFO("Communication started");
    }
    else
    {
      ROS_ERROR("Connection failed");
    }
  }

  Hokuyo3dNode()
    : pnh_("~")
    , timestamp_base_(0)
    , timer_(io_, boost::posix_time::milliseconds(500))
  {
    pnh_.param("horizontal_interlace", horizontal_interlace_, 4);
    if (horizontal_interlace_ <= 0 || horizontal_interlace_ > 20)
    {
      ROS_ERROR("Invalid horizontal_interlace value %d", horizontal_interlace_);
      ros::shutdown();
    }
    pnh_.param("vertical_interlace", vertical_interlace_, 1);
    if (vertical_interlace_ <= 0 || vertical_interlace_ > 10)
    {
      ROS_ERROR("Invalid vertical_interlace_ value %d", vertical_interlace_);
      ros::shutdown();
    }
    pnh_.param("ip", ip_, std::string("192.168.0.10"));
    pnh_.param("port", port_, 10940);
    pnh_.param("frame_id", frame_id_, std::string("hokuyo3d"));
    pnh_.param("imu_frame_id", imu_frame_id_, frame_id_ + "_imu");
    pnh_.param("mag_frame_id", mag_frame_id_, frame_id_ + "_mag");
    pnh_.param("range_min", range_min_, 0.0);
    set_auto_reset_ = pnh_.hasParam("auto_reset");
    pnh_.param("auto_reset", auto_reset_, false);

    pnh_.param("allow_jump_back", allow_jump_back_, false);

    driver_.setTimeout(2.0);
    ROS_INFO("Connecting to %s", ip_.c_str());
    driver_.registerCallback(boost::bind(&Hokuyo3dNode::cbPoint, this, _1, _2, _3, _4, _5, _6));
    driver_.registerAuxCallback(boost::bind(&Hokuyo3dNode::cbAux, this, _1, _2, _3, _4));
    driver_.registerPingCallback(boost::bind(&Hokuyo3dNode::cbPing, this, _1, _2));
    driver_.registerErrorCallback(boost::bind(&Hokuyo3dNode::cbError, this, _1, _2, _3));
    field_ = 0;
    frame_ = 0;
    line_ = 0;

    sensor_msgs::ChannelFloat32 channel;
    channel.name = std::string("intensity");
    cloud_.channels.push_back(channel);

    cloud2_frame_.height = 1;
    cloud2_field_.height = 1;
    cloud2_line_.height = 1;
    cloud2_frame_.is_bigendian = false;
    cloud2_field_.is_bigendian = false;
    cloud2_line_.is_bigendian = false;
    cloud2_frame_.is_dense = false;
    cloud2_field_.is_dense = false;
    cloud2_line_.is_dense = false;
    sensor_msgs::PointCloud2Modifier pc2_frame_modifier(cloud2_frame_);
    sensor_msgs::PointCloud2Modifier pc2_field_modifier(cloud2_field_);
    sensor_msgs::PointCloud2Modifier pc2_line_modifier(cloud2_line_);
    pc2_frame_modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1,
                                      sensor_msgs::PointField::FLOAT32, "z", 1, sensor_msgs::PointField::FLOAT32,
                                      "intensity", 1, sensor_msgs::PointField::FLOAT32);
    pc2_field_modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1,
                                      sensor_msgs::PointField::FLOAT32, "z", 1, sensor_msgs::PointField::FLOAT32,
                                      "intensity", 1, sensor_msgs::PointField::FLOAT32);
    pc2_line_modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1,
                                      sensor_msgs::PointField::FLOAT32, "z", 1, sensor_msgs::PointField::FLOAT32,
                                      "intensity", 1, sensor_msgs::PointField::FLOAT32);

    pub_imu_ = pnh_.advertise<sensor_msgs::Imu>("imu", 5);
    pub_mag_ = pnh_.advertise<sensor_msgs::MagneticField>("mag", 5);

    enable_pc_ = enable_pc2_ = false;
    ros::SubscriberStatusCallback cb_con = boost::bind(&Hokuyo3dNode::cbSubscriber, this);

    boost::lock_guard<boost::mutex> lock(connect_mutex_);
    pub_pc_ = pnh_.advertise<sensor_msgs::PointCloud>("hokuyo_cloud", 5, cb_con, cb_con);
    pub_pc2_frame_ = pnh_.advertise<sensor_msgs::PointCloud2>("hokuyo_cloud2_frame", 5, cb_con, cb_con);
    pub_pc2_field_ = pnh_.advertise<sensor_msgs::PointCloud2>("hokuyo_cloud2_field", 5, cb_con, cb_con);
    pub_pc2_line_ = pnh_.advertise<sensor_msgs::PointCloud2>("hokuyo_cloud2_line", 5, cb_con, cb_con);

    // Start communication with the sensor
    driver_.connect(ip_.c_str(), port_, boost::bind(&Hokuyo3dNode::cbConnect, this, _1));
  }

  ~Hokuyo3dNode()
  {
    driver_.requestAuxData(false);
    driver_.requestData(true, false);
    driver_.requestData(false, false);
    driver_.poll();
    ROS_INFO("Communication stoped");
  }

  void cbSubscriber()
  {
    boost::lock_guard<boost::mutex> lock(connect_mutex_);
    if (pub_pc_.getNumSubscribers() > 0)
    {
      enable_pc_ = true;
      ROS_DEBUG("PointCloud output enabled");
    }
    else
    {
      enable_pc_ = false;
      ROS_DEBUG("PointCloud output disabled");
    }
    if (pub_pc2_frame_.getNumSubscribers() > 0 || pub_pc2_field_.getNumSubscribers() > 0 || pub_pc2_line_.getNumSubscribers() > 0)
    {
      enable_pc2_ = true;
      ROS_DEBUG("PointCloud2 output enabled");
    }
    else
    {
      enable_pc2_ = false;
      ROS_DEBUG("PointCloud2 output disabled");
    }
  }

  bool poll()
  {
    if (driver_.poll())
    {
      return true;
    }
    ROS_INFO("Connection closed");
    return false;
  }

  void cbTimer(const boost::system::error_code& error)
  {
    if (error)
      return;

    if (!ros::ok())
    {
      driver_.stop();
    }
    else
    {
      timer_.expires_at(
          timer_.expires_at() +
          boost::posix_time::milliseconds(500));
      timer_.async_wait(boost::bind(&Hokuyo3dNode::cbTimer, this, _1));
    }
  }

  void spin()
  {
    timer_.async_wait(boost::bind(&Hokuyo3dNode::cbTimer, this, _1));
    boost::thread thread(boost::bind(&boost::asio::io_service::run, &io_));

    ros::AsyncSpinner spinner(1);
    spinner.start();
    driver_.spin();
    spinner.stop();
    timer_.cancel();
    ROS_INFO("Connection closed");
  }
  void ping()
  {
    driver_.requestPing();
    time_ping_ = ros::Time::now();
  }

protected:
  ros::NodeHandle pnh_;
  ros::Publisher pub_pc_;
  ros::Publisher pub_pc2_frame_;
  ros::Publisher pub_pc2_field_;
  ros::Publisher pub_pc2_line_;
  ros::Publisher pub_imu_;
  ros::Publisher pub_mag_;
  vssp::VsspDriver driver_;
  sensor_msgs::PointCloud cloud_;
  sensor_msgs::PointCloud2 cloud2_frame_;
  sensor_msgs::PointCloud2 cloud2_field_;
  sensor_msgs::PointCloud2 cloud2_line_;
  sensor_msgs::Imu imu_;
  sensor_msgs::MagneticField mag_;

  bool enable_pc_;
  bool enable_pc2_;
  bool allow_jump_back_;
  boost::mutex connect_mutex_;

  ros::Time time_ping_;
  ros::Time timestamp_base_;
  std::deque<ros::Time> timestamp_base_buffer_;
  ros::Time imu_stamp_last_;
  ros::Time mag_stamp_last_;
  ros::Time cloud_stamp_last_;
  ros::Time cloud2_frame_stamp_last_;
  ros::Time cloud2_field_stamp_last_;
  ros::Time cloud2_line_stamp_last_;

  boost::asio::io_service io_;
  boost::asio::deadline_timer timer_;

  int field_;
  int frame_;
  int line_;

  enum PublishCycle
  {
    CYCLE_FIELD,
    CYCLE_FRAME,
    CYCLE_LINE
  };
  PublishCycle cycle_;
  std::string ip_;
  int port_;
  int horizontal_interlace_;
  int vertical_interlace_;
  double range_min_;
  std::string frame_id_;
  std::string imu_frame_id_;
  std::string mag_frame_id_;
  bool auto_reset_;
  bool set_auto_reset_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hokuyo3d");
  Hokuyo3dNode node;

  node.spin();

  return 1;
}
