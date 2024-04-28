cmake_minimum_required(VERSION 3.5)

project(GOLEM CXX)

add_executable(${PROJECT_NAME}
    Main.cpp

)

find_package(OpenCV 4 REQUIRED)

target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${OpenCV_INCLUDE_DIRS}
        "include"
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        ${OpenCV_LIBS}
)