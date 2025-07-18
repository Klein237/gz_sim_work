cmake_minimum_required(VERSION 3.5)

# Following directive defines the project name.
project(ros_gz_gazebo)


# Following directives find required packages and load their configuration.
# The 'set' directive defines a variable (e.g. 'GZ_PLUGIN_VER').
# Such variables can be used lateron in the CMakeLists.txt file.
find_package(ament_cmake REQUIRED)
find_package(ros_gz_description REQUIRED)

find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
find_package(gz-common5 REQUIRED COMPONENTS profiler)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

find_package(gz-transport13 REQUIRED)
set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})


# Harmonic
if("$ENV{GZ_VERSION}" STREQUAL "harmonic")
  find_package(gz-sim8 REQUIRED)
  set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})
  message(STATUS "Compiling against Gazebo Harmonic")
# Default to Garden
else()
  find_package(gz-sim8 REQUIRED)
  set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})
  message(STATUS "Compiling against Gazebo Garden")
endif()


# Following 'add_library' directive defines a library target named 'BasicSystem'.
# The 'SHARED' keyword indicates that a shared library should be compiled, and
# is followed by the list of source files for the target.
add_library(BasicSystem
  SHARED
  src/BasicSystem.cc
)

# Following 'target_include_directories' directive specifies 'include' as the
# include directory to use when compiling the 'BasicSystem' target.
# The 'PRIVATE' keyword specifies that this directive will populate the
# INCLUDE_DIRECTORIES property for the 'BasicSystem' target.
target_include_directories(
  BasicSystem PRIVATE include
)

# Following 'target_link_libraries' directive specifies to use the 
# gz-sim library when linking the 'BasicSystem' target.
# The 'PRIVATE' keyword stipulates that the gz-sim library will not
# automatically be included if the 'BasicSystem' target were to
# be linked to anoter target. 
# ${GZ_SIM_VER} is substituted by the value that is was set to above.
target_link_libraries(BasicSystem PRIVATE
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})


# Following directives similarly specify another target: 'FullSystem'.
add_library(FullSystem
  SHARED
  src/FullSystem.cc
)

target_include_directories(
  FullSystem PRIVATE include
)

target_link_libraries(FullSystem PRIVATE
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})



# Following 'install' directive ensures that the compiled libraries
# of the 'BasicSystem' and 'FullSystem' targets will be copied
# to the subfolder 'lib/ros_gz_example_gazebo' of the install directory.
install(
  TARGETS BasicSystem FullSystem
  DESTINATION lib/${PROJECT_NAME}
)


# --- ActorVelocityControlSystem Plugin ------------------------

# 1) Crée la librairie partagée
add_library(ActorVelocityControlSystem
  SHARED
  src/ActorVelocityControlSystem.cc
)

# 2) Indique où trouver vos headers
target_include_directories(ActorVelocityControlSystem
  PRIVATE include
)

# 3) Lie la bibliothèque aux bonnes dépendances
target_link_libraries(ActorVelocityControlSystem
  PRIVATE
    # Gazebo Sim core
    gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
    # Gazebo Transport
    gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
    # Le loader de plugins
    gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
)



# 4) (Optionnel) C++17
set_property(TARGET ActorVelocityControlSystem PROPERTY CXX_STANDARD 17)

# 5) Enregistre le plugin pour qu'il soit chargé dynamiquement
#    via gz-plugin2 (register component)
# gz_register_plugin(
#   TARGET ActorVelocityControlSystem
#   # nom de la librairie (sera injecté dans <plugin filename="...">)
#   PLUGIN_NAME ActorVelocityControlSystem
#   # nom complet de la classe au sein du plugin
#   CLASS_NAME ros_gz_gazebo::ActorVelocityControlSystem
# )

# 6) Installe la librairie au même endroit que les autres
install(
  TARGETS ActorVelocityControlSystem
  DESTINATION lib/${PROJECT_NAME}
)


# Following 'install' directive ensures that the 'worlds' subfolder
# will be copied to the 'share/ros_gz_example_gazebo/worlds'
# subfolder of the installation directory.
install(
  DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)


# Following directives are used when testing.
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


# Following hooks are used to ensure that the correct environment variables
# will be set by exectuting 'source install/setup.bash' after compilation.
# When using this template for your project, change the filenames of the
# files in the 'hooks' folder, to correspond to your project name.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")


# Following directive configures ament based on the previous directives, and should
# typically be the last directive in the CMakeLists.txt file.
ament_package()
