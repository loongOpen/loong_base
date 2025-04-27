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
#include"share_mem.h"
#include"data_center.h"
#include<vector>
#include"iopack.h"

namespace Task{

class periodicTaskClass{
public:
	periodicTaskClass(){};
	~periodicTaskClass(){};

	virtual void init(const string &name,float dt)=0;
	virtual bool step(){return running;}
	
	virtual int start(int priority, int cpu)final;
	virtual int join()final;
	virtual void stop()final{running=0;}

	virtual float getDt()final{return dt;}
protected:
	periodicTaskClass(const periodicTaskClass&)=delete;
	periodicTaskClass& operator=(const periodicTaskClass&)=delete;
	bool running=1;
	unsigned long threadId=0;
	string taskName;
	float dt;
};


}//namespace

