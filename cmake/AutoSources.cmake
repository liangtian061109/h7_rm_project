# -----------------------------------------------------------------------------
# AutoSources.cmake —— 自动收集用户代码的编译"插件"
#
# 作用:
#   1. 递归扫描给定目录下所有 .c 文件，自动加进 target 的源文件列表；
#   2. 递归扫描所有 .h 文件，把每个头文件所在目录自动加进 include 路径；
#   3. 使用 CONFIGURE_DEPENDS（CMake >= 3.12）：新增/删除 .c/.h 文件后，
#      下次构建时 Ninja 会自动重新执行 cmake，无需手动改 CMakeLists；
#   4. 自动跳过 build/、.git/ 等中间目录，避免把生成文件编译进去；
#   5. 去重，已由 CubeMX 加入的源文件不会被重复添加。
#
# 用法（根 CMakeLists.txt 的 USER 区）:
#   include(${CMAKE_SOURCE_DIR}/cmake/AutoSources.cmake)
#   auto_add_sources(${CMAKE_PROJECT_NAME} APP BSP modules)
#
# 注意:
#   * 只收集 .c（本工程是纯 C）；要支持 C++ 时把 GLOB 里的 *.c 改成 *.c *.cpp
#   * 目录用相对路径时相对于调用该函数的 CMakeLists.txt 所在目录
# -----------------------------------------------------------------------------

# 需要跳过的路径（正则，注意 Windows 下已统一成 / 分隔符）
set(AUTO_SOURCES_SKIP_REGEX "/(build|Build|out|CMakeFiles|\\.git|\\.vscode|\\.settings|\\.workbuddy-ai)/"
    CACHE STRING "AutoSources: 递归收集源码时需要跳过的路径正则")

function(auto_add_sources TARGET_NAME)
  if(NOT TARGET ${TARGET_NAME})
    message(FATAL_ERROR "[AutoSources] 目标不存在: ${TARGET_NAME}")
  endif()
  if(ARGC LESS 2)
    message(FATAL_ERROR "[AutoSources] 用法: auto_add_sources(<target> <目录> [目录...])")
  endif()

  # 目标里已有的源文件（CubeMX 生成的那些），用于去重
  get_target_property(_existing_sources ${TARGET_NAME} SOURCES)
  set(_existing_abs "")
  if(_existing_sources)
    foreach(_src IN LISTS _existing_sources)
      if(IS_ABSOLUTE "${_src}")
        list(APPEND _existing_abs "${_src}")
      else()
        get_filename_component(_src_abs "${_src}" ABSOLUTE)
        list(APPEND _existing_abs "${_src_abs}")
      endif()
    endforeach()
  endif()

  set(_sources "")
  set(_includes "")
  set(_scanned_dirs 0)

  foreach(_item IN LISTS ARGN)
    if(IS_ABSOLUTE "${_item}")
      set(_base "${_item}")
    else()
      set(_base "${CMAKE_CURRENT_SOURCE_DIR}/${_item}")
    endif()

    if(NOT IS_DIRECTORY "${_base}")
      message(WARNING "[AutoSources] 目录不存在，已跳过: ${_base}")
      continue()
    endif()

    file(TO_CMAKE_PATH "${_base}" _base)
    math(EXPR _scanned_dirs "${_scanned_dirs} + 1")

    file(GLOB_RECURSE _c_files CONFIGURE_DEPENDS "${_base}/*.c")
    file(GLOB_RECURSE _h_files CONFIGURE_DEPENDS "${_base}/*.h")

    foreach(_file IN LISTS _c_files)
      file(TO_CMAKE_PATH "${_file}" _file)
      if(_file MATCHES "${AUTO_SOURCES_SKIP_REGEX}")
        continue()
      endif()
      list(FIND _existing_abs "${_file}" _idx)
      if(NOT _idx EQUAL -1)
        continue()
      endif()
      list(APPEND _sources "${_file}")
    endforeach()

    foreach(_file IN LISTS _h_files)
      file(TO_CMAKE_PATH "${_file}" _file)
      if(_file MATCHES "${AUTO_SOURCES_SKIP_REGEX}")
        continue()
      endif()
      get_filename_component(_dir "${_file}" DIRECTORY)
      list(APPEND _includes "${_dir}")
    endforeach()

    # 目录本身也加进 include 路径（头文件直接放根目录时也能找到）
    list(APPEND _includes "${_base}")
  endforeach()

  list(REMOVE_DUPLICATES _sources)
  list(REMOVE_DUPLICATES _includes)

  if(_sources)
    target_sources(${TARGET_NAME} PRIVATE ${_sources})
    list(LENGTH _sources _count)
    message(STATUS "[AutoSources] ${TARGET_NAME}: 自动添加 ${_count} 个 .c 文件（扫描 ${_scanned_dirs} 个目录）")
  else()
    message(STATUS "[AutoSources] ${TARGET_NAME}: 没有发现 .c 文件（扫描 ${_scanned_dirs} 个目录）")
  endif()

  if(_includes)
    target_include_directories(${TARGET_NAME} PRIVATE ${_includes})
    list(LENGTH _includes _count)
    message(STATUS "[AutoSources] ${TARGET_NAME}: 自动添加 ${_count} 个头文件目录")
  endif()
endfunction()
