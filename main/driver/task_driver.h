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


=====================================================*/
#pragma once
#include"task_base.h"

namespace Task{

class driverTaskClass:public periodicTaskClass{
public:
	driverTaskClass();
	~driverTaskClass();

	void init(const string &name,float dt)override final;
	bool step()override final;
	void disable();
private:
	driverTaskClass(const driverTaskClass&)=delete;
	driverTaskClass& operator=(const driverTaskClass&)=delete;
	class impClass;
	impClass&imp;
};


}//namespace

