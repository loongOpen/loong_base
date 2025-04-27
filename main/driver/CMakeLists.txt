#[[
Copyright 2025 国家地方共建人形机器人创新中心/人形机器人（上海）有限公司, https://www.openloong.net
Thanks for the open biped control project Nabo: https://github.com/tryingfly/nabo

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
       http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License. 

=========== ***doc description @ yyp*** ===========

=====================================================]]

set(loong_driver loong_driver_${tgtArch})

file(GLOB_RECURSE srcFiles *.cpp)
add_executable(${loong_driver} ${srcFiles})
target_link_directories(${loong_driver} PRIVATE ${CMAKE_SOURCE_DIR}/module/loong_driver_sdk)

# if(sim_mode) #仿真采用udp-mujoco，不link实机sdk
# 	message("driver采用udp")
# 	add_library(loong_driver_sim ${CMAKE_SOURCE_DIR}/main/driver_sim/loong_driver_sdk_sim.cpp)
# 	target_link_libraries(loong_driver_sim libTask)
# 	set(loong_driver_sdk loong_driver_sim)
# else()
# 	message("driver采用sdk")
	set(loong_driver_sdk loong_driver_sdk_${tgtArch} joint_map_${tgtArch})
# endif()

target_link_libraries(${loong_driver} libTask pthread ${loong_driver_sdk})

install(TARGETS ${loong_driver} DESTINATION .)
