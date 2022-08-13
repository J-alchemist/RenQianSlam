#设置WORK_SPACE_PATH的值 要与global_defination.h.in里@  @之间的对应
set(WORK_SPACE_PATH ${PROJECT_SOURCE_DIR})

#configure_file：将global_defination.h.in里面@  @之间的变量
configure_file (
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/global_defination/global_defination.h.in
  ${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}/global_defination/global_defination.h)
include_directories(${PROJECT_BINARY_DIR}/include) 


