cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(pixhawk_imu_heartbeat)

target_sources(app PRIVATE inc/serial_com.h
                       inc/mavlink_tx.h
                       inc/mavlink_rx.h
                       src/main.c
                       src/serial_com.c
                       src/mavlink_tx.c
                       src/mavlink_rx.c)
