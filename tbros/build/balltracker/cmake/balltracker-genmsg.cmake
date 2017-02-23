# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "balltracker: 2 messages, 0 services")

set(MSG_I_FLAGS "-Iballtracker:/home/kc/sandbox/triniBot/tbros/src/balltracker/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(balltracker_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/kc/sandbox/triniBot/tbros/src/balltracker/msg/ballcoords.msg" NAME_WE)
add_custom_target(_balltracker_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "balltracker" "/home/kc/sandbox/triniBot/tbros/src/balltracker/msg/ballcoords.msg" ""
)

get_filename_component(_filename "/home/kc/sandbox/triniBot/tbros/src/balltracker/msg/command.msg" NAME_WE)
add_custom_target(_balltracker_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "balltracker" "/home/kc/sandbox/triniBot/tbros/src/balltracker/msg/command.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(balltracker
  "/home/kc/sandbox/triniBot/tbros/src/balltracker/msg/ballcoords.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/balltracker
)
_generate_msg_cpp(balltracker
  "/home/kc/sandbox/triniBot/tbros/src/balltracker/msg/command.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/balltracker
)

### Generating Services

### Generating Module File
_generate_module_cpp(balltracker
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/balltracker
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(balltracker_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(balltracker_generate_messages balltracker_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kc/sandbox/triniBot/tbros/src/balltracker/msg/ballcoords.msg" NAME_WE)
add_dependencies(balltracker_generate_messages_cpp _balltracker_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kc/sandbox/triniBot/tbros/src/balltracker/msg/command.msg" NAME_WE)
add_dependencies(balltracker_generate_messages_cpp _balltracker_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(balltracker_gencpp)
add_dependencies(balltracker_gencpp balltracker_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS balltracker_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(balltracker
  "/home/kc/sandbox/triniBot/tbros/src/balltracker/msg/ballcoords.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/balltracker
)
_generate_msg_eus(balltracker
  "/home/kc/sandbox/triniBot/tbros/src/balltracker/msg/command.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/balltracker
)

### Generating Services

### Generating Module File
_generate_module_eus(balltracker
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/balltracker
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(balltracker_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(balltracker_generate_messages balltracker_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kc/sandbox/triniBot/tbros/src/balltracker/msg/ballcoords.msg" NAME_WE)
add_dependencies(balltracker_generate_messages_eus _balltracker_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kc/sandbox/triniBot/tbros/src/balltracker/msg/command.msg" NAME_WE)
add_dependencies(balltracker_generate_messages_eus _balltracker_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(balltracker_geneus)
add_dependencies(balltracker_geneus balltracker_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS balltracker_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(balltracker
  "/home/kc/sandbox/triniBot/tbros/src/balltracker/msg/ballcoords.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/balltracker
)
_generate_msg_lisp(balltracker
  "/home/kc/sandbox/triniBot/tbros/src/balltracker/msg/command.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/balltracker
)

### Generating Services

### Generating Module File
_generate_module_lisp(balltracker
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/balltracker
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(balltracker_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(balltracker_generate_messages balltracker_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kc/sandbox/triniBot/tbros/src/balltracker/msg/ballcoords.msg" NAME_WE)
add_dependencies(balltracker_generate_messages_lisp _balltracker_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kc/sandbox/triniBot/tbros/src/balltracker/msg/command.msg" NAME_WE)
add_dependencies(balltracker_generate_messages_lisp _balltracker_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(balltracker_genlisp)
add_dependencies(balltracker_genlisp balltracker_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS balltracker_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(balltracker
  "/home/kc/sandbox/triniBot/tbros/src/balltracker/msg/ballcoords.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/balltracker
)
_generate_msg_nodejs(balltracker
  "/home/kc/sandbox/triniBot/tbros/src/balltracker/msg/command.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/balltracker
)

### Generating Services

### Generating Module File
_generate_module_nodejs(balltracker
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/balltracker
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(balltracker_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(balltracker_generate_messages balltracker_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kc/sandbox/triniBot/tbros/src/balltracker/msg/ballcoords.msg" NAME_WE)
add_dependencies(balltracker_generate_messages_nodejs _balltracker_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kc/sandbox/triniBot/tbros/src/balltracker/msg/command.msg" NAME_WE)
add_dependencies(balltracker_generate_messages_nodejs _balltracker_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(balltracker_gennodejs)
add_dependencies(balltracker_gennodejs balltracker_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS balltracker_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(balltracker
  "/home/kc/sandbox/triniBot/tbros/src/balltracker/msg/ballcoords.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/balltracker
)
_generate_msg_py(balltracker
  "/home/kc/sandbox/triniBot/tbros/src/balltracker/msg/command.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/balltracker
)

### Generating Services

### Generating Module File
_generate_module_py(balltracker
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/balltracker
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(balltracker_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(balltracker_generate_messages balltracker_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kc/sandbox/triniBot/tbros/src/balltracker/msg/ballcoords.msg" NAME_WE)
add_dependencies(balltracker_generate_messages_py _balltracker_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kc/sandbox/triniBot/tbros/src/balltracker/msg/command.msg" NAME_WE)
add_dependencies(balltracker_generate_messages_py _balltracker_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(balltracker_genpy)
add_dependencies(balltracker_genpy balltracker_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS balltracker_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/balltracker)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/balltracker
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(balltracker_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/balltracker)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/balltracker
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(balltracker_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/balltracker)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/balltracker
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(balltracker_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/balltracker)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/balltracker
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(balltracker_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/balltracker)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/balltracker\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/balltracker
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(balltracker_generate_messages_py std_msgs_generate_messages_py)
endif()
