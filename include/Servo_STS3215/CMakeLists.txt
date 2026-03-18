cmake_minimum_required(VERSION 3.10)
project(servo_move)

set(CMAKE_CXX_STANDARD 17)

add_executable(servo_move MENU.cpp)

find_package(yaml-cpp REQUIRED)
target_link_libraries(servo_move yaml-cpp)


# SCServo headers
target_include_directories(servo_move PRIVATE
    /home/aqid/SCServo_Linux
)

# SCServo library
target_link_libraries(servo_move
    /home/aqid/SCServo_Linux/build/libSCServo.a
)