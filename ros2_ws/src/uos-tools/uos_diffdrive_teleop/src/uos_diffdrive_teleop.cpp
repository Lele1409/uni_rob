/*
 *
 * Copyright (C) 2015 University of Osnabrück, Germany
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * uos_diffdrive_teleop.cpp
 *
 *  Created on: 16.02.2015
 *      Author: Sebastian Pütz <spuetz@uos.de>
 */

#include <uos_diffdrive_teleop.h>

Teleop::Teleop() : rclcpp::Node("teleop_node")
{
  
  get_parameter_or("max_vel", max_vel, 1.0);           // max velocity
  get_parameter_or("max_rot_vel", max_rot_vel, 2.0);   // max rot velocity

  //forward acceleration parameters
  get_parameter_or("acc_ahead_pos", acc_y.pos, 0.4);
  get_parameter_or("acc_ahead_neg", acc_y.neg, 0.8);
  get_parameter_or("acc_ahead_stop", acc_y.stop, 2.0);

  //rotational acceleration parameters
  get_parameter_or("acc_rot_pos", acc_x.pos, 1.4);
  get_parameter_or("acc_rot_neg", acc_x.neg, 1.8);
  get_parameter_or("acc_rot_stop", acc_x.stop, 2.6);

  //update rates
  get_parameter_or("update_velocity_rate", update_velocity_rate, 0.01);
  get_parameter_or("update_inputs_rate", update_inputs_rate, 0.05);

  vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

  vel_timer = this->create_wall_timer(std::chrono::duration<double>(update_velocity_rate), std::bind(&Teleop::updateVelocity, this));
  key_timer = this->create_wall_timer(std::chrono::duration<double>(update_inputs_rate), std::bind(&Teleop::updateInputs, this));

  velo.x = 0;
  velo.y = 0;

  lastUpdate = now();
}

void Teleop::updateVelocity(){
  
  rclcpp::Duration delta = (now() - lastUpdate);
  double left = in.left;
  double forwards = in.forwards;

  left = std::min(1.0, left);
  left = std::max(-1.0, left);
  forwards = std::min(1.0, forwards);
  forwards = std::max(-1.0, forwards);

  velo.y = adaptVelocity(delta.seconds(), velo.y, forwards, acc_y.stop, acc_y.neg, acc_y.pos);
  velo.x = adaptVelocity(delta.seconds(), velo.x, left, acc_x.stop, acc_x.neg, acc_x.pos);

  // velocity limits by intensity
  velo.dyn_limit_y = max_vel * forwards;
  velo.dyn_limit_x = max_rot_vel * left;

  // observe the limits for the forward part
  if((forwards < 0 && velo.y < velo.dyn_limit_y)
      || (forwards > 0 && velo.y > velo.dyn_limit_y))
    velo.y = velo.dyn_limit_y;

  // observe the limits for the rotational part
  if((left < 0 && velo.x < velo.dyn_limit_x)
      || (left > 0 && velo.x > velo.dyn_limit_x))
    velo.x = velo.dyn_limit_x;

  // set command 
  vel_cmd.linear.x = velo.y;
  vel_cmd.angular.z = velo.x;

  //publish command
  if(std::abs(vel_cmd.linear.x) > EPSILON_VELO || std::abs(vel_cmd.angular.z) > EPSILON_VELO) {
    vel_pub->publish(vel_cmd);
  }
  // TODO apply velocity by service call
}

void Teleop::updateInputs(){
  if(!in.updated){
    in.forwards = 0;
    in.left = 0;
  }
  in.updated = false;
}

double Teleop::adaptVelocity(
    double delta_time, // delta time
    double velocity,  // current velocity
    double factor,    // intensity factor
    double acc_stop,  // active deceleration 
    double acc_neg,   // deceleartion, if no button is pressed
    double acc_pos)   // positive acceleration
{

  double new_velocity = velocity;

  if(factor != 0) {
    if(factor > 0) {
      // active decelerate
      if(velocity < 0)
        return (new_velocity += acc_stop * factor * delta_time) > -EPSILON_VELO ? 0 : new_velocity;
      // accelerate
      else 
        return new_velocity + acc_pos * factor * delta_time;
    } else {
      // active decelerate
      if(velocity > 0)
        return (new_velocity += acc_stop * factor * delta_time) < EPSILON_VELO ? 0 : new_velocity;
      // accelerate
      else
        return new_velocity + acc_pos * factor * delta_time;
    }
  }
  else {
    // nothing pressed -> decelerate
    if(velocity > 0)
      return (new_velocity += -acc_neg * delta_time) < EPSILON_VELO ? 0 : new_velocity;
    else if(velocity < 0)
      return (new_velocity += acc_neg * delta_time) > -EPSILON_VELO ? 0 : new_velocity;
    return 0;
  }
}
