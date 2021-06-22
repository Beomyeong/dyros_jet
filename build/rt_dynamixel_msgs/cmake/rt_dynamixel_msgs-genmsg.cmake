# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "rt_dynamixel_msgs: 3 messages, 2 services")

set(MSG_I_FLAGS "-Irt_dynamixel_msgs:/home/beom/catkin_ws/src/rt_dynamixel_msgs/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(rt_dynamixel_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/beom/catkin_ws/src/rt_dynamixel_msgs/msg/JointState.msg" NAME_WE)
add_custom_target(_rt_dynamixel_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rt_dynamixel_msgs" "/home/beom/catkin_ws/src/rt_dynamixel_msgs/msg/JointState.msg" ""
)

get_filename_component(_filename "/home/beom/catkin_ws/src/rt_dynamixel_msgs/srv/ModeSetting.srv" NAME_WE)
add_custom_target(_rt_dynamixel_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rt_dynamixel_msgs" "/home/beom/catkin_ws/src/rt_dynamixel_msgs/srv/ModeSetting.srv" ""
)

get_filename_component(_filename "/home/beom/catkin_ws/src/rt_dynamixel_msgs/srv/MotorSetting.srv" NAME_WE)
add_custom_target(_rt_dynamixel_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rt_dynamixel_msgs" "/home/beom/catkin_ws/src/rt_dynamixel_msgs/srv/MotorSetting.srv" ""
)

get_filename_component(_filename "/home/beom/catkin_ws/src/rt_dynamixel_msgs/msg/JointSet.msg" NAME_WE)
add_custom_target(_rt_dynamixel_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rt_dynamixel_msgs" "/home/beom/catkin_ws/src/rt_dynamixel_msgs/msg/JointSet.msg" ""
)

get_filename_component(_filename "/home/beom/catkin_ws/src/rt_dynamixel_msgs/msg/Error.msg" NAME_WE)
add_custom_target(_rt_dynamixel_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rt_dynamixel_msgs" "/home/beom/catkin_ws/src/rt_dynamixel_msgs/msg/Error.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(rt_dynamixel_msgs
  "/home/beom/catkin_ws/src/rt_dynamixel_msgs/msg/Error.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rt_dynamixel_msgs
)
_generate_msg_cpp(rt_dynamixel_msgs
  "/home/beom/catkin_ws/src/rt_dynamixel_msgs/msg/JointSet.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rt_dynamixel_msgs
)
_generate_msg_cpp(rt_dynamixel_msgs
  "/home/beom/catkin_ws/src/rt_dynamixel_msgs/msg/JointState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rt_dynamixel_msgs
)

### Generating Services
_generate_srv_cpp(rt_dynamixel_msgs
  "/home/beom/catkin_ws/src/rt_dynamixel_msgs/srv/MotorSetting.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rt_dynamixel_msgs
)
_generate_srv_cpp(rt_dynamixel_msgs
  "/home/beom/catkin_ws/src/rt_dynamixel_msgs/srv/ModeSetting.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rt_dynamixel_msgs
)

### Generating Module File
_generate_module_cpp(rt_dynamixel_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rt_dynamixel_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(rt_dynamixel_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(rt_dynamixel_msgs_generate_messages rt_dynamixel_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/beom/catkin_ws/src/rt_dynamixel_msgs/msg/JointState.msg" NAME_WE)
add_dependencies(rt_dynamixel_msgs_generate_messages_cpp _rt_dynamixel_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/beom/catkin_ws/src/rt_dynamixel_msgs/srv/ModeSetting.srv" NAME_WE)
add_dependencies(rt_dynamixel_msgs_generate_messages_cpp _rt_dynamixel_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/beom/catkin_ws/src/rt_dynamixel_msgs/srv/MotorSetting.srv" NAME_WE)
add_dependencies(rt_dynamixel_msgs_generate_messages_cpp _rt_dynamixel_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/beom/catkin_ws/src/rt_dynamixel_msgs/msg/JointSet.msg" NAME_WE)
add_dependencies(rt_dynamixel_msgs_generate_messages_cpp _rt_dynamixel_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/beom/catkin_ws/src/rt_dynamixel_msgs/msg/Error.msg" NAME_WE)
add_dependencies(rt_dynamixel_msgs_generate_messages_cpp _rt_dynamixel_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rt_dynamixel_msgs_gencpp)
add_dependencies(rt_dynamixel_msgs_gencpp rt_dynamixel_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rt_dynamixel_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(rt_dynamixel_msgs
  "/home/beom/catkin_ws/src/rt_dynamixel_msgs/msg/Error.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rt_dynamixel_msgs
)
_generate_msg_eus(rt_dynamixel_msgs
  "/home/beom/catkin_ws/src/rt_dynamixel_msgs/msg/JointSet.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rt_dynamixel_msgs
)
_generate_msg_eus(rt_dynamixel_msgs
  "/home/beom/catkin_ws/src/rt_dynamixel_msgs/msg/JointState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rt_dynamixel_msgs
)

### Generating Services
_generate_srv_eus(rt_dynamixel_msgs
  "/home/beom/catkin_ws/src/rt_dynamixel_msgs/srv/MotorSetting.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rt_dynamixel_msgs
)
_generate_srv_eus(rt_dynamixel_msgs
  "/home/beom/catkin_ws/src/rt_dynamixel_msgs/srv/ModeSetting.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rt_dynamixel_msgs
)

### Generating Module File
_generate_module_eus(rt_dynamixel_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rt_dynamixel_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(rt_dynamixel_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(rt_dynamixel_msgs_generate_messages rt_dynamixel_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/beom/catkin_ws/src/rt_dynamixel_msgs/msg/JointState.msg" NAME_WE)
add_dependencies(rt_dynamixel_msgs_generate_messages_eus _rt_dynamixel_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/beom/catkin_ws/src/rt_dynamixel_msgs/srv/ModeSetting.srv" NAME_WE)
add_dependencies(rt_dynamixel_msgs_generate_messages_eus _rt_dynamixel_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/beom/catkin_ws/src/rt_dynamixel_msgs/srv/MotorSetting.srv" NAME_WE)
add_dependencies(rt_dynamixel_msgs_generate_messages_eus _rt_dynamixel_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/beom/catkin_ws/src/rt_dynamixel_msgs/msg/JointSet.msg" NAME_WE)
add_dependencies(rt_dynamixel_msgs_generate_messages_eus _rt_dynamixel_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/beom/catkin_ws/src/rt_dynamixel_msgs/msg/Error.msg" NAME_WE)
add_dependencies(rt_dynamixel_msgs_generate_messages_eus _rt_dynamixel_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rt_dynamixel_msgs_geneus)
add_dependencies(rt_dynamixel_msgs_geneus rt_dynamixel_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rt_dynamixel_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(rt_dynamixel_msgs
  "/home/beom/catkin_ws/src/rt_dynamixel_msgs/msg/Error.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rt_dynamixel_msgs
)
_generate_msg_lisp(rt_dynamixel_msgs
  "/home/beom/catkin_ws/src/rt_dynamixel_msgs/msg/JointSet.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rt_dynamixel_msgs
)
_generate_msg_lisp(rt_dynamixel_msgs
  "/home/beom/catkin_ws/src/rt_dynamixel_msgs/msg/JointState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rt_dynamixel_msgs
)

### Generating Services
_generate_srv_lisp(rt_dynamixel_msgs
  "/home/beom/catkin_ws/src/rt_dynamixel_msgs/srv/MotorSetting.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rt_dynamixel_msgs
)
_generate_srv_lisp(rt_dynamixel_msgs
  "/home/beom/catkin_ws/src/rt_dynamixel_msgs/srv/ModeSetting.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rt_dynamixel_msgs
)

### Generating Module File
_generate_module_lisp(rt_dynamixel_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rt_dynamixel_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(rt_dynamixel_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(rt_dynamixel_msgs_generate_messages rt_dynamixel_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/beom/catkin_ws/src/rt_dynamixel_msgs/msg/JointState.msg" NAME_WE)
add_dependencies(rt_dynamixel_msgs_generate_messages_lisp _rt_dynamixel_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/beom/catkin_ws/src/rt_dynamixel_msgs/srv/ModeSetting.srv" NAME_WE)
add_dependencies(rt_dynamixel_msgs_generate_messages_lisp _rt_dynamixel_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/beom/catkin_ws/src/rt_dynamixel_msgs/srv/MotorSetting.srv" NAME_WE)
add_dependencies(rt_dynamixel_msgs_generate_messages_lisp _rt_dynamixel_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/beom/catkin_ws/src/rt_dynamixel_msgs/msg/JointSet.msg" NAME_WE)
add_dependencies(rt_dynamixel_msgs_generate_messages_lisp _rt_dynamixel_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/beom/catkin_ws/src/rt_dynamixel_msgs/msg/Error.msg" NAME_WE)
add_dependencies(rt_dynamixel_msgs_generate_messages_lisp _rt_dynamixel_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rt_dynamixel_msgs_genlisp)
add_dependencies(rt_dynamixel_msgs_genlisp rt_dynamixel_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rt_dynamixel_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(rt_dynamixel_msgs
  "/home/beom/catkin_ws/src/rt_dynamixel_msgs/msg/Error.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rt_dynamixel_msgs
)
_generate_msg_nodejs(rt_dynamixel_msgs
  "/home/beom/catkin_ws/src/rt_dynamixel_msgs/msg/JointSet.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rt_dynamixel_msgs
)
_generate_msg_nodejs(rt_dynamixel_msgs
  "/home/beom/catkin_ws/src/rt_dynamixel_msgs/msg/JointState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rt_dynamixel_msgs
)

### Generating Services
_generate_srv_nodejs(rt_dynamixel_msgs
  "/home/beom/catkin_ws/src/rt_dynamixel_msgs/srv/MotorSetting.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rt_dynamixel_msgs
)
_generate_srv_nodejs(rt_dynamixel_msgs
  "/home/beom/catkin_ws/src/rt_dynamixel_msgs/srv/ModeSetting.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rt_dynamixel_msgs
)

### Generating Module File
_generate_module_nodejs(rt_dynamixel_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rt_dynamixel_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(rt_dynamixel_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(rt_dynamixel_msgs_generate_messages rt_dynamixel_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/beom/catkin_ws/src/rt_dynamixel_msgs/msg/JointState.msg" NAME_WE)
add_dependencies(rt_dynamixel_msgs_generate_messages_nodejs _rt_dynamixel_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/beom/catkin_ws/src/rt_dynamixel_msgs/srv/ModeSetting.srv" NAME_WE)
add_dependencies(rt_dynamixel_msgs_generate_messages_nodejs _rt_dynamixel_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/beom/catkin_ws/src/rt_dynamixel_msgs/srv/MotorSetting.srv" NAME_WE)
add_dependencies(rt_dynamixel_msgs_generate_messages_nodejs _rt_dynamixel_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/beom/catkin_ws/src/rt_dynamixel_msgs/msg/JointSet.msg" NAME_WE)
add_dependencies(rt_dynamixel_msgs_generate_messages_nodejs _rt_dynamixel_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/beom/catkin_ws/src/rt_dynamixel_msgs/msg/Error.msg" NAME_WE)
add_dependencies(rt_dynamixel_msgs_generate_messages_nodejs _rt_dynamixel_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rt_dynamixel_msgs_gennodejs)
add_dependencies(rt_dynamixel_msgs_gennodejs rt_dynamixel_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rt_dynamixel_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(rt_dynamixel_msgs
  "/home/beom/catkin_ws/src/rt_dynamixel_msgs/msg/Error.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rt_dynamixel_msgs
)
_generate_msg_py(rt_dynamixel_msgs
  "/home/beom/catkin_ws/src/rt_dynamixel_msgs/msg/JointSet.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rt_dynamixel_msgs
)
_generate_msg_py(rt_dynamixel_msgs
  "/home/beom/catkin_ws/src/rt_dynamixel_msgs/msg/JointState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rt_dynamixel_msgs
)

### Generating Services
_generate_srv_py(rt_dynamixel_msgs
  "/home/beom/catkin_ws/src/rt_dynamixel_msgs/srv/MotorSetting.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rt_dynamixel_msgs
)
_generate_srv_py(rt_dynamixel_msgs
  "/home/beom/catkin_ws/src/rt_dynamixel_msgs/srv/ModeSetting.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rt_dynamixel_msgs
)

### Generating Module File
_generate_module_py(rt_dynamixel_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rt_dynamixel_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(rt_dynamixel_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(rt_dynamixel_msgs_generate_messages rt_dynamixel_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/beom/catkin_ws/src/rt_dynamixel_msgs/msg/JointState.msg" NAME_WE)
add_dependencies(rt_dynamixel_msgs_generate_messages_py _rt_dynamixel_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/beom/catkin_ws/src/rt_dynamixel_msgs/srv/ModeSetting.srv" NAME_WE)
add_dependencies(rt_dynamixel_msgs_generate_messages_py _rt_dynamixel_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/beom/catkin_ws/src/rt_dynamixel_msgs/srv/MotorSetting.srv" NAME_WE)
add_dependencies(rt_dynamixel_msgs_generate_messages_py _rt_dynamixel_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/beom/catkin_ws/src/rt_dynamixel_msgs/msg/JointSet.msg" NAME_WE)
add_dependencies(rt_dynamixel_msgs_generate_messages_py _rt_dynamixel_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/beom/catkin_ws/src/rt_dynamixel_msgs/msg/Error.msg" NAME_WE)
add_dependencies(rt_dynamixel_msgs_generate_messages_py _rt_dynamixel_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rt_dynamixel_msgs_genpy)
add_dependencies(rt_dynamixel_msgs_genpy rt_dynamixel_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rt_dynamixel_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rt_dynamixel_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rt_dynamixel_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(rt_dynamixel_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rt_dynamixel_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rt_dynamixel_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(rt_dynamixel_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rt_dynamixel_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rt_dynamixel_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(rt_dynamixel_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rt_dynamixel_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rt_dynamixel_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(rt_dynamixel_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rt_dynamixel_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rt_dynamixel_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rt_dynamixel_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(rt_dynamixel_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
