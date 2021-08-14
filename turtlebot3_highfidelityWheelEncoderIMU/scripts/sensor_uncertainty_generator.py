#!/usr/bin/env python


# https://numpy.org/doc/stable/reference/random/generated/numpy.random.poisson.html
from numpy.random import poisson
import rospy, json, threading, os, traceback
from std_msgs.msg import Float32MultiArray


def imu_noise_publish(publisher):
    print("\nIMU noise publisher started.\n")
    high_msg = Float32MultiArray(data = imu_noise_high_array)
    low_msg = Float32MultiArray(data = imu_noise_low_array)

    counter = 0
    start_time = 0

    for _ in range(2):
        publisher.publish(low_msg)
        rate.sleep()

    while rospy.is_shutdown:
        imu_poisson = poisson(imu_num_of_error_occurences / time_interval)
        if imu_poisson >= 1:
            print("IMU error activated: " + str(imu_poisson))
            publisher.publish(high_msg)
            start_time = counter

        if counter - start_time == imu_noise_duration:
            print("IMU error deactivated: " + str(imu_poisson))
            publisher.publish(low_msg)
        
        rate.sleep()
        counter += 1

def diffd_noise_publish(publisher):
    print("\nDiffdrive noise publisher started.\n")
    high_msg = Float32MultiArray(data = diffd_noise_high_array)
    low_msg = Float32MultiArray(data = diffd_noise_low_array)

    for _ in range(2):
        publisher.publish(low_msg)
        rate.sleep()

    counter = 0
    start_time = 0

    while rospy.is_shutdown:
        diffd_poisson = poisson(diff_drive_num_of_error_occurences / time_interval)
        if diffd_poisson >= 1:
            print("Diffdrive error activated : " + str(diffd_poisson))
            publisher.publish(high_msg)
            start_time = counter

        if counter - start_time == diff_drive_noise_duration:
            print("Diffdrive error deactivated : " + str(diffd_poisson))
            publisher.publish(low_msg)

        rate.sleep()
        counter += 1

def main():
    if obj["imu"] == "True":
        imu_gaussian_noise_publisher = rospy.Publisher("imu/set_noise", Float32MultiArray, queue_size=1)
        imu_thread = threading.Thread(target=imu_noise_publish, args=(imu_gaussian_noise_publisher, ))
        imu_thread.start()

    if obj["diff_drive"] == "True":
        diff_drive_gaussian_noise_publisher = rospy.Publisher("diff_drive/set_noise", Float32MultiArray, queue_size=1)
        diffd_thread = threading.Thread(target=diffd_noise_publish, args=(diff_drive_gaussian_noise_publisher, ))
        diffd_thread.start()
    
if __name__ == "__main__":
    rospy.init_node("sensor_uncertainty", anonymous=True)

    curWD = os.path.dirname(__file__)
    file_name = curWD + "/sensor_uncertainty_config.json"

    try:
        with open(file_name, 'r') as file:
            obj = json.load(file)
        
        imu_num_of_error_occurences = float(obj["imu_num_of_error_occurences"])
        diff_drive_num_of_error_occurences = float(obj["diff_drive_num_of_error_occurences"])

        time_interval = int(obj["time_interval"])
        rate = rospy.Rate(int(obj["rate"]))

        imu_noise_duration = int(obj["imu_noise_duration"])
        diff_drive_noise_duration = int(obj["diff_drive_noise_duration"])

        imu_noise_low_array = [ float(obj["imu_orientation_x_low_noise"]), 
                                float(obj["imu_orientation_y_low_noise"]), 
                                float(obj["imu_orientation_z_low_noise"]),
                                float(obj["imu_orientation_w_low_noise"]),
                                float(obj["imu_angular_velocity_x_low_noise"]), 
                                float(obj["imu_angular_velocity_y_low_noise"]), 
                                float(obj["imu_angular_velocity_z_low_noise"]),
                                float(obj["imu_linear_acceleration_x_low_noise"]), 
                                float(obj["imu_linear_acceleration_y_low_noise"]), 
                                float(obj["imu_linear_acceleration_z_low_noise"])]

        imu_noise_high_array = [float(obj["imu_orientation_x_high_noise"]), 
                                float(obj["imu_orientation_y_high_noise"]), 
                                float(obj["imu_orientation_z_high_noise"]),
                                float(obj["imu_orientation_w_high_noise"]),
                                float(obj["imu_angular_velocity_x_high_noise"]),
                                float(obj["imu_angular_velocity_y_high_noise"]), 
                                float(obj["imu_angular_velocity_z_high_noise"]),
                                float(obj["imu_linear_acceleration_x_high_noise"]), 
                                float(obj["imu_linear_acceleration_y_high_noise"]), 
                                float(obj["imu_linear_acceleration_z_high_noise"])]
    
        diffd_noise_low_array = [float(obj["diff_drive_left_wheel_low_noise"]), float(obj["diff_drive_right_wheel_low_noise"])]
        diffd_noise_high_array = [float(obj["diff_drive_left_wheel_high_noise"]), float(obj["diff_drive_right_wheel_high_noise"])]

    except Exception:
        print("\nError occured while reading 'sensor_uncertainty_config.json' file.\n")
        traceback.print_exc()
        print("\nNode is exiting!\n")
        exit()


    try:
        main()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    