#include <iostream>
#include <thread>
#include <chrono>

#include "tf3/buffer_core.h"
#include "tf3/compat.h"
#include "tf3/LinearMath/Transform.h"

using namespace tf3;

static void printTransform(const TransformStampedMsg & t)
{
  std::cout << "Transform: [" << t.header.frame_id << "] -> [" << t.child_frame_id << "]\n";
  std::cout << "  stamp: " << t.header.stamp.toSec() << "\n";
  std::cout << "  translation: (" << t.transform.translation.x << ", " << t.transform.translation.y << ", " << t.transform.translation.z << ")\n";
  std::cout << "  rotation: (" << t.transform.rotation.x << ", " << t.transform.rotation.y << ", " << t.transform.rotation.z << ", " << t.transform.rotation.w << ")\n";
}

int main()
{
  // Create a buffer with 10 seconds cache
  BufferCore buffer(tf3::Duration(10));

  // Create a transform from "world" -> "robot"
  TransformStampedMsg world_to_robot;
  world_to_robot.header.stamp = tf3::Time::fromSec(1.0);
  world_to_robot.header.frame_id = "world";
  world_to_robot.child_frame_id = "robot";
  world_to_robot.transform.translation.x = 1.0;
  world_to_robot.transform.translation.y = 2.0;
  world_to_robot.transform.translation.z = 0.0;
  // identity rotation
  world_to_robot.transform.rotation.x = 0.0;
  world_to_robot.transform.rotation.y = 0.0;
  world_to_robot.transform.rotation.z = 0.0;
  world_to_robot.transform.rotation.w = 1.0;

  // Insert the transform into the buffer (this simulates broadcasting)
  buffer.setTransform(world_to_robot, "example_authority", false);

  std::cout << "Inserted world->robot transform:\n";
  printTransform(world_to_robot);

  // Create another transform robot -> sensor
  TransformStampedMsg robot_to_sensor;
  robot_to_sensor.header.stamp = tf3::Time::fromSec(1.0);
  robot_to_sensor.header.frame_id = "robot";
  robot_to_sensor.child_frame_id = "sensor";
  robot_to_sensor.transform.translation.x = 0.1;
  robot_to_sensor.transform.translation.y = 0.0;
  robot_to_sensor.transform.translation.z = 0.2;
  robot_to_sensor.transform.rotation.x = 0.0;
  robot_to_sensor.transform.rotation.y = 0.0;
  robot_to_sensor.transform.rotation.z = 0.0;
  robot_to_sensor.transform.rotation.w = 1.0;

  buffer.setTransform(robot_to_sensor, "example_authority", false);
  std::cout << "Inserted robot->sensor transform:\n";
  printTransform(robot_to_sensor);

  // Query: get transform from sensor -> world (composed path)
  try
  {
    // lookupTransform(target_frame, source_frame, time)
    // returns the transform that maps a point in `source_frame` into `target_frame`.
    TransformStampedMsg sensor_to_world = buffer.lookupTransform("world", "sensor", tf3::Time());
    std::cout << "Lookup: sensor -> world (composed):\n";
    printTransform(sensor_to_world);
  }
  catch (const std::exception & ex)
  {
    std::cerr << "lookupTransform failed: " << ex.what() << std::endl;
  }

  // Simulate broadcasting updates over time (update robot position)
  for (int i = 0; i < 3; ++i)
  {
    double tsec = 2.0 + i * 1.0;
    world_to_robot.header.stamp = tf3::Time::fromSec(tsec);
    world_to_robot.transform.translation.x += 0.5; // robot moves in x
    buffer.setTransform(world_to_robot, "example_authority", false);
    std::cout << "Updated world->robot at t=" << tsec << "\n";
    // lookup at latest
    TransformStampedMsg latest = buffer.lookupTransform("world", "sensor", tf3::Time());
    std::cout << "Latest composed transform (sensor->world):\n";
    printTransform(latest);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  return 0;
}
