cmake_minimum_required(VERSION 3.4)

project(libfranka-examples CXX)

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_LIST_DIR}/../cmake)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Poco REQUIRED COMPONENTS Foundation)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_library(examples_common STATIC
  examples_common.cpp
)

add_library(sigmoid_clbf STATIC
  hc_CLBFs/sigmoid_clbf/sigmoid_clbf.cpp
)

set(QPOASES_ROOT "/home/cdsl/qpOASES")
set(QPOASES_INCLUDE_DIR "${QPOASES_ROOT}/include")
set(QPOASES_LIB "${QPOASES_ROOT}/build/libs/libqpOASES.a")

target_link_libraries(examples_common PUBLIC Franka::Franka Eigen3::Eigen3 ${QPOASES_LIB})
target_include_directories(examples_common PUBLIC ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(sigmoid_clbf PUBLIC Eigen3::Eigen3)
target_include_directories(sigmoid_clbf PUBLIC ${CMAKE_CURRENT_LIST_DIR}/hc_CLBFs)

set(EXAMPLES
  cartesian_impedance_control
  communication_test
  echo_robot_state
  force_control
  generate_cartesian_pose_motion
  generate_cartesian_velocity_motion
  generate_consecutive_motions
  generate_elbow_motion
  generate_joint_position_motion
  generate_joint_velocity_motion
  grasp_object
  joint_impedance_control
  joint_point_to_point_motion
  motion_with_control
  print_joint_poses
  vacuum_object
  robot_state_recording
  )

set(ys_QPCs
  QPC_controller
  QPC_controller_time
  QPC_controller_3dof
  )

set(hc_CLBFs
  my_custom_controller_hp
  hp_custom_cartesian_FL
  hp_cartesian_FL_CLBF
  )

foreach(example ${EXAMPLES})
  add_executable(${example} ${example}.cpp)
  target_link_libraries(${example} Franka::Franka examples_common Eigen3::Eigen3)
endforeach()

# ys_QPCs (examples/ys_QPCs/ 아래에 .cpp가 있음)
foreach(example ${ys_QPCs})
  add_executable(${example} ys_QPCs/${example}.cpp)
  target_link_libraries(${example} Franka::Franka examples_common Eigen3::Eigen3)
endforeach()

# hc_CLBFs (examples/hc_CLBFs/ 아래에 .cpp가 있음)
foreach(example ${hc_CLBFs})
  add_executable(${example} hc_CLBFs/${example}.cpp)
  target_link_libraries(${example} Franka::Franka examples_common Eigen3::Eigen3 sigmoid_clbf)
endforeach()

target_link_libraries(joint_impedance_control Threads::Threads)
target_link_libraries(motion_with_control Poco::Foundation)

include(GNUInstallDirs)
install(TARGETS ${EXAMPLES} ${ys_QPCs} ${hc_CLBFs}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
