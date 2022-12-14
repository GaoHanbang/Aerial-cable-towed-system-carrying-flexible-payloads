/****************************************************************************
 *
 * Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/*!
 * @@file microRTPS_timesync.cpp
 * @@brief source code for time sync implementation
 */

#include <time.h>
#include <cmath>
#include <iostream>

#include "microRTPS_timesync.h"

TimeSync::TimeSync(bool debug, rclcpp::Publisher<px4_msgs::msg::Timesync>::SharedPtr ts_publisher_)
    : _offset_ns(-1),
      _skew_ns_per_sync(0.0),
      _num_samples(0),
      _request_reset_counter(0),
      _last_msg_seq(0),
      _last_remote_msg_seq(0),
      _debug(debug),
      ts_publisher(ts_publisher_)
{ }

TimeSync::~TimeSync() = default;

void TimeSync::spin() {
    px4_msgs::msg::Timesync msg = newTimesyncMsg();
	ts_publisher->publish(msg);
}

void TimeSync::reset() {
	_num_samples = 0;
	_request_reset_counter = 0;
}

int64_t TimeSync::getTimeNSec() {
	auto time = std::chrono::steady_clock::now();
	return std::chrono::time_point_cast<std::chrono::nanoseconds>(time).time_since_epoch().count();
}

int64_t TimeSync::getTimeUSec() {
	auto time = std::chrono::steady_clock::now();
	return std::chrono::time_point_cast<std::chrono::microseconds>(time).time_since_epoch().count();
}

bool TimeSync::addMeasurement(int64_t local_t1_ns, int64_t remote_t2_ns, int64_t local_t3_ns) {
	int64_t rtti = local_t3_ns - local_t1_ns;
    if (rtti_store.size() == 10) // Keeping 10 elements to compute mean over 10 samples (1s)
        rtti_store.pop();
    rtti_store.push(rtti);

	// assume rtti is evenly split both directions
	int64_t remote_t3_ns = remote_t2_ns + rtti / 2ll;

	int64_t measurement_offset = remote_t3_ns - local_t3_ns;

	if (_request_reset_counter > REQUEST_RESET_COUNTER_THRESHOLD) {
		reset();
		if (_debug) std::cout << "\033[1;33m[ micrortps__timesync ]\tTimesync clock changed, resetting\033[0m" << std::endl;
	}

        if (_num_samples == 0) {
                updateOffset(measurement_offset);
                _skew_ns_per_sync = 0;
        }

	if (_num_samples >= WINDOW_SIZE) {
		if (std::abs(measurement_offset - _offset_ns.load()) > TRIGGER_RESET_THRESHOLD_NS) {
			_request_reset_counter++;
			if (_debug) std::cout << "\033[1;33m[ micrortps__timesync ]\tTimesync offset outlier, discarding\033[0m" << std::endl;
			return false;
		} else {
			_request_reset_counter = 0;
		}
	}

	// ignore if rtti > 50ms
	if (rtti > 50ll * 1000ll * 1000ll) {
		if (_debug) std::cout << "\033[1;33m[ micrortps__timesync ]\tRTTI too high for timesync: " << rtti / (1000ll * 1000ll) << "ms\033[0m" << std::endl;
		return false;
	}

	double alpha = ALPHA_FINAL;
	double beta = BETA_FINAL;

	if (_num_samples < WINDOW_SIZE) {
		double schedule = (double)_num_samples / WINDOW_SIZE;
		double s = 1. - exp(.5 * (1. - 1. / (1. - schedule)));
		alpha = (1. - s) * ALPHA_INITIAL + s * ALPHA_FINAL;
		beta = (1. - s) * BETA_INITIAL + s * BETA_FINAL;
	}

	int64_t offset_prev = _offset_ns.load();
	updateOffset(static_cast<int64_t>((_skew_ns_per_sync + _offset_ns.load()) * (1. - alpha) +
					  measurement_offset * alpha));
	_skew_ns_per_sync =
	    static_cast<int64_t>(beta * (_offset_ns.load() - offset_prev) + (1. - beta) * _skew_ns_per_sync);

	_num_samples++;

	return true;
}

void TimeSync::processTimesyncMsg(px4_msgs::msg::Timesync *msg) {
	if (getMsgSysID(msg) == 1 && getMsgSeq(msg) != _last_remote_msg_seq) {
                _last_remote_msg_seq = getMsgSeq(msg);

		if (getMsgTC1(msg) > 0) {
			if (!addMeasurement(getMsgTS1(msg), getMsgTC1(msg), getTimeNSec())) {
				if (_debug) std::cerr << "\033[1;33m[ micrortps__timesync ]\tOffset not updated\033[0m" << std::endl;
			}

		} else if (getMsgTC1(msg) == 0) {
			setMsgTimestamp(msg, getTimeUSec());
			setMsgSysID(msg, 0);
			setMsgSeq(msg, getMsgSeq(msg) + 1);
			setMsgTC1(msg, getTimeNSec());

			ts_publisher->publish(*msg);
		}
	}
}

px4_msgs::msg::Timesync TimeSync::newTimesyncMsg() {
    px4_msgs::msg::Timesync msg;

	setMsgTimestamp(&msg, getTimeUSec());
	setMsgSysID(&msg, 0);
	setMsgSeq(&msg, _last_msg_seq);
	setMsgTC1(&msg, 0);
	setMsgTS1(&msg, getTimeNSec());

	_last_msg_seq++;

	return msg;
}

uint64_t TimeSync::getMeanDelay() {
    uint8_t delay_size = rtti_store.size();
    uint64_t mean = 0;
    while (!rtti_store.empty())
    {
        mean += rtti_store.front();
        rtti_store.pop();
    }
    // Delay is returned in us
    if (delay_size > 0)
        return mean/delay_size/1000;
    else
        return 999999999;
}