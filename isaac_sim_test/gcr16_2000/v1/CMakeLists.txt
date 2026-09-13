cmake_minimum_required(VERSION 3.8)
project(gcr16_2000_v1)
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf launch rviz meshes scripts
  DESTINATION share/${PROJECT_NAME}
  PATTERN "export_meshes.sh" PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)
ament_package()
