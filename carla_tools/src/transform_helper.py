
import tf.transformations
import numpy as np
from sensor_msgs.msg import Imu, PointCloud2
from copy import deepcopy

carla_to_ros = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ], dtype=np.float32)

# ros_to_rovio_world = np.array([
#     [0.0, -1.0, 0.0, 0.0],
#     [1.0, 0.0, 0.0, 0.0],
#     [0.0, 0.0, 1.0, 0.0],
#     [0.0, 0.0, 0.0, 1.0]
# ])
#
# ros_to_rovio = np.array([
#             [0.0, -1.0, 0.0, 0.0],
#             [0.0, 0.0, -1.0, 0.0],
#             [1.0, 0.0, 0.0, 0.0],
#             [0.0, 0.0, 0.0, 1.0]
#         ], dtype=np.float32)

ros_to_loam = np.array([
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ], dtype=np.float32)

ros_to_velodyne = np.array([
            [0.0, -1.0, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ], dtype=np.float32)

# ros_to_rovio = tf.transformations.inverse_matrix(ros_to_rovio)
# ros_to_rovio_world = tf.transformations.inverse_matrix(ros_to_rovio_world)
ros_to_loam = tf.transformations.inverse_matrix(ros_to_loam)
ros_to_velodyne = tf.transformations.inverse_matrix(ros_to_velodyne)

def transform_covariance(covariance, transform):
    cov = np.reshape(np.array(covariance), [3, 3])
    return np.matmul(transform[0:3, 0:3], np.matmul(cov, np.transpose(transform[0:3, 0:3])))


def transform_imu(msg, rotation):
    transform = tf.transformations.quaternion_matrix(rotation)
    orig_accel = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z, 1.0])
    # Convert to right-handed rotation
    orig_gyro = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z, 1.0])
    orig_orientation = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])

    ros_orientation = tf.transformations.quaternion_multiply(rotation, orig_orientation)

    ros_accel = np.matmul(transform, orig_accel)
    ros_accel_cov = transform_covariance(msg.linear_acceleration_covariance, transform)
    ros_gyro = np.matmul(transform, orig_gyro)
    ros_gyro_cov = transform_covariance(msg.angular_velocity_covariance, transform)
    # ros_orientation = np.matmul(quaternion_transform, orig_orientation)

    ros_orientation_cov = transform_covariance(msg.orientation_covariance, transform)
    ros_msg = Imu()
    ros_msg.header.stamp = msg.header.stamp
    ros_msg.linear_acceleration.x = ros_accel[0]
    ros_msg.linear_acceleration.y = ros_accel[1]
    ros_msg.linear_acceleration.z = ros_accel[2]
    ros_msg.linear_acceleration_covariance = np.reshape(ros_accel_cov, [-1])
    ros_msg.angular_velocity.x = ros_gyro[0]
    ros_msg.angular_velocity.y = ros_gyro[1]
    ros_msg.angular_velocity.z = ros_gyro[2]
    ros_msg.angular_velocity_covariance = np.reshape(ros_gyro_cov, [-1])
    ros_msg.orientation.x = ros_orientation[0]
    ros_msg.orientation.y = ros_orientation[1]
    ros_msg.orientation.z = ros_orientation[2]
    ros_msg.orientation.w = ros_orientation[3]
    ros_msg.orientation_covariance = np.reshape(ros_orientation_cov, [-1])
    return ros_msg

def transform_pointcloud2(msg, transform):
    transform = transform.astype(np.float32)
    new_msg = deepcopy(msg)
    data = np.fromstring(new_msg.data, dtype=np.float32)
    orig_shape = data.shape
    data = np.reshape(data, [-1, 3]).transpose()
    ones = np.ones([1, data.shape[1]], data.dtype)
    data = np.concatenate((data, ones), axis=0)
    data = np.matmul(transform, data)
    data = data.transpose()[:, 0:3]
    data = data.reshape([-1])
    new_msg.data = data.tostring()
    return new_msg
