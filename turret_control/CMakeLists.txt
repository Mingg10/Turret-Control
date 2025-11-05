cmake_minimum_required(VERSION 3.16)
project(pi3hat_motor_test)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# # Enable FetchContent
# include(FetchContent)

# # Fetch pi3hat using official release tag
# FetchContent_Declare(
#   pi3hat
#   GIT_REPOSITORY https://github.com/mjbots/pi3hat.git
#   GIT_TAG 0.1-20241002  # From official release [3][4]
# )

# # Raspberry Pi optimizations
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8-a+crc -mtune=cortex-a72")

# # Configure and build dependencies
# FetchContent_MakeAvailable(pi3hat)

# Add executable for main motor test
add_executable(motor_test
  src/main.cpp
  src/pi3hat.cpp
  src/pdb.cpp
  src/cubemars_control.cpp
)

# Include directories
target_include_directories(motor_test PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include

)

# Link libraries: On Ubuntu server, remove "raspberrypi" (which is typically provided on Raspberry Pi OS)
target_link_libraries(motor_test PRIVATE
  #pi3hat
  pigpio
  bcm_host
)

# Add executable for spool wire control test
add_executable(spool_test
  src/spool_test.cpp
  src/pi3hat.cpp
  src/pdb.cpp
  src/cubemars_control.cpp
)

# Include directories for spool_test
target_include_directories(spool_test PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include

)

# Link libraries for spool_test
target_link_libraries(spool_test PRIVATE
  #pi3hat
  pigpio
  bcm_host
)

# Install rules
install(TARGETS motor_test spool_test DESTINATION bin)
