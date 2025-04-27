/*
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

============ ***doc description @ yyp*** ============
关节映射通用接口
update 电机到关节
dwdate 关节到电机
=====================================================*/
#pragma once
#include"eigen.h"

namespace JntMap{

extern "C" void update(vecXf &pos,vecXf &vel,vecXf &tor);
extern "C" void dwdate(vecXf &pos,vecXf &vel,vecXf &tor);

}//namespace
