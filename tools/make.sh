#!/bin/bash
# Copyright 2025 国家地方共建人形机器人创新中心/人形机器人（上海）有限公司, https://www.openloong.net
# Thanks for the open biped control project Nabo: https://github.com/tryingfly/nabo

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#         http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# ============ ***doc description @ yyp*** ============
# 多分文件夹可以大幅减少重编译耗时
# ${1:0:1} 第1个参数的第0位开始长度为1的字符
#======================================================

if [ ${#1} != 2 ]; then
	echo ==!!参数不匹配!!==
	echo xs: x64 仿真
	echo xr: x64 实机
	echo as: aarch64 仿真
	echo ar: aarch64 实机
	echo =================
	exit
fi

cd ..
if [ ${1:0:1} == 'x' ];then
	if [ ${1:1:1} == 's' ]; then
		if [ ! -e "build_x64_sim" ]; then
			mkdir build_x64_sim
		fi
		cd build_x64_sim
		cmake .. -Dcpu_x64=1 -Dsim_mode=1
		make -j6 install
		# echo === 本机x64，复制.so到mujoco ===
		# cp ../nabo_output/libshare_manipulation.so ../../loong_mujoco_manipulation/module/share_sim/
	else
		if [ ! -e "build_x64_robot" ]; then
			mkdir build_x64_robot
		fi
		cd build_x64_robot
		cmake ..  -Dcpu_x64=1 -Dsim_mode=0
		make -j6 install
		echo ==== 实机x64，复制.so到ssh ===
		cd ../tools/
		./update_exe.sh x
	fi
else
	if [ ${1:1:1} == 's' ]; then
		if [ ! -e "build_a64_sim" ]; then
			mkdir build_a64_sim
		fi
		cd build_a64_sim
		cmake ..  -Dcpu_x64=0 -Dsim_mode=1
		make -j6 install
		# echo === 实机x64，复制.so到mujoco ===
		# cp ../nabo_output/libnabo_a64.so ../../loong_mujoco/module/share_sim/
	else
		if [ ! -e "build_a64_robot" ]; then
			mkdir build_a64_robot
		fi
		cd build_a64_robot
		cmake ..  -Dcpu_x64=0 -Dsim_mode=0
		make -j6 install
		echo ==== 实机aarch64，复制.so到ssh ===
		cd ../tools/
		# ./update_exe.sh a
	fi
fi

