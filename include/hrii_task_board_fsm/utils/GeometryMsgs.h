#ifndef __HRII_TASK_BOARD_FSM_UTILS_GEOMETRY_MSGS_H__
#define __HRII_TASK_BOARD_FSM_UTILS_GEOMETRY_MSGS_H__

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>

namespace geometry_msgs {

inline geometry_msgs::Transform toTransform(const geometry_msgs::Pose& pose)
{
    geometry_msgs::Transform transform;
    transform.translation.x = pose.position.x;
    transform.translation.y = pose.position.y;
    transform.translation.z = pose.position.z;
    transform.rotation = pose.orientation;
    return transform;
}

inline geometry_msgs::Pose toPose(const geometry_msgs::Transform& transform)
{
    geometry_msgs::Pose pose;
    pose.position.x = transform.translation.x;
    pose.position.y = transform.translation.y;
    pose.position.z = transform.translation.z;
    pose.orientation = transform.rotation;
    return pose;
}
} // namespace geometry_msgs

#endif // __HRII_TASK_BOARD_FSM_UTILS_GEOMETRY_MSGS_H__