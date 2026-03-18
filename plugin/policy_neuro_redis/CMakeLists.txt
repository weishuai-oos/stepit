cmake_minimum_required(VERSION 3.16)
project(stepit_plugin_policy_neuro_redis)

find_package(PkgConfig QUIET)
if (NOT PkgConfig_FOUND)
  message(STATUS "Skipped plugin '${PROJECT_NAME}': Missing PkgConfig.")
  return()
endif ()

pkg_check_modules(HIREDIS QUIET hiredis)
if (NOT HIREDIS_FOUND)
  message(STATUS "Skipped plugin '${PROJECT_NAME}': Missing hiredis.")
  return()
endif ()

stepit_add_plugin(${PROJECT_NAME}
    SOURCES
      src/field_subscriber_redis.cpp
      src/redis_client.cpp
    INCLUDES
      include
      ${HIREDIS_INCLUDE_DIRS}
    LINK_LIBS
      ${HIREDIS_LIBRARIES}
    LINK_DIRS
      ${HIREDIS_LIBRARY_DIRS}
    FLAGS
      -Wall -Wextra -Wpedantic -Wno-unused-parameter
      ${HIREDIS_CFLAGS_OTHER}
    DEPENDS
      policy_neuro
)
