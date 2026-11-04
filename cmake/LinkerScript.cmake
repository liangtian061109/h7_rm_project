# -----------------------------------------------------------------------------
# LinkerScript.cmake —— 用工程自有链接脚本替换 CubeMX 生成的链接脚本
#
# 背景:
#   cmake/gcc-arm-none-eabi.cmake 里写死了
#       -T "${CMAKE_SOURCE_DIR}/STM32H723XG_FLASH.ld"
#   而 STM32H723XG_FLASH.ld 是 CubeMX 每次 Generate Code 都会覆盖的生成文件，
#   不能直接在里面加 .dma_buffer 段（加了下次生成就丢）。
#   所以工程自己维护一份 STM32H723XG_Project.ld（= 生成脚本 + .dma_buffer 段，
#   放在 RAM_D1，供 DMA 访问），再用本插件在链接时替换掉生成脚本。
#
# 做法（和 MermaidFAR/H7_BSP 的 h7_linker.cmake 思路一致）:
#   1. 从 CMAKE_EXE_LINKER_FLAGS 里摘掉 -T 生成脚本那一项；
#   2. 给可执行目标追加 target_link_options(-T<自有脚本>)；
#   3. LINK_DEPENDS 指向自有脚本，脚本一改就重新链接；
#   4. 万一以后 CubeMX 改了工具链模板、匹配不到原来的 -T，直接 FATAL_ERROR，
#      防止静默退回生成脚本（那样 .dma_buffer 就变成孤立段，DMA 到处访问垃圾）。
#
# 用法（根 CMakeLists.txt 的 USER 区）:
#   include(${CMAKE_SOURCE_DIR}/cmake/LinkerScript.cmake)
#   override_linker_script(${CMAKE_PROJECT_NAME}
#                          "${CMAKE_SOURCE_DIR}/STM32H723XG_Project.ld")
# -----------------------------------------------------------------------------

function(override_linker_script TARGET_NAME LD_FILE)
  if(NOT TARGET ${TARGET_NAME})
    message(FATAL_ERROR "[LinkerScript] 目标不存在: ${TARGET_NAME}")
  endif()
  if(NOT EXISTS "${LD_FILE}")
    message(FATAL_ERROR "[LinkerScript] 链接脚本不存在: ${LD_FILE}")
  endif()

  set(_flags "${CMAKE_EXE_LINKER_FLAGS}")

  # 匹配形如 -T "D:/xxx/STM32H723XG_FLASH.ld" 或 -T/xxx/STM32H723XG_FLASH.ld
  string(REGEX MATCHALL
         "-T[ \t]+\"?[^ \t+\"]*STM32H723XG_FLASH\\.ld\"?"
         _matched_flags "${_flags}")

  if(NOT _matched_flags)
    message(FATAL_ERROR
      "[LinkerScript] 没在 CMAKE_EXE_LINKER_FLAGS 里找到 -T ...STM32H723XG_FLASH.ld。\n"
      "  当前值: ${_flags}\n"
      "  CubeMX 重新生成过工具链模板？请检查 cmake/gcc-arm-none-eabi.cmake，\n"
      "  再按需修改 cmake/LinkerScript.cmake 的正则。")
  endif()

  foreach(_flag IN LISTS _matched_flags)
    string(REPLACE "${_flag}" "" _flags "${_flags}")
  endforeach()
  string(STRIP "${_flags}" _flags)

  # 修改回调用方（根 CMakeLists.txt）的目录作用域，链接命令从这里取参数
  set(CMAKE_EXE_LINKER_FLAGS "${_flags}" PARENT_SCOPE)

  target_link_options(${TARGET_NAME} PRIVATE "-T${LD_FILE}")
  set_target_properties(${TARGET_NAME} PROPERTIES LINK_DEPENDS "${LD_FILE}")

  get_filename_component(_ld_name "${LD_FILE}" NAME)
  message(STATUS "[LinkerScript] 已用工程自有链接脚本覆盖生成脚本: ${_ld_name}")
endfunction()
