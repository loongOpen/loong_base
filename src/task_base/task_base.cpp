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
#include"task_base.h"
#include<pthread.h>
#include<thread>
#include<chrono>

namespace Task{

struct argStruct{
	const string name;
	periodicTaskClass *task;
	int cpuId;
};

void setCpu(const string &name,int cpuId){
	cpu_set_t cpuSet;
	CPU_ZERO(&cpuSet);
	CPU_SET(cpuId,&cpuSet);
	if(pthread_setaffinity_np(pthread_self(),sizeof(cpuSet),&cpuSet)<0){
		throw runtime_error("绑定cpu失败");
	}else{
		CPU_ZERO(&cpuSet);
		if(pthread_getaffinity_np(pthread_self(),sizeof(cpuSet),&cpuSet)<0){
			throw runtime_error("获取绑定cpu失败");
		}else{
			int cpuNums=8;
			For(cpuNums){
				if(CPU_ISSET(i,&cpuSet)){
					printL(name,"线程绑定到cpu-",i);
				}
			}
		}
	}
}
void *loop(void* argP){
	argStruct &arg=*((argStruct*)argP);
	periodicTaskClass &task=*arg.task;
	int us=task.getDt()*1e6;

	printL(arg.name,"线程开始");
	setCpu(arg.name,arg.cpuId);

	auto refClock=chrono::high_resolution_clock().now();
	while(task.step()){
		refClock+=chrono::microseconds(us);//微秒
		this_thread::sleep_until(refClock);
	}
	delete &arg;
	return 0;
}
//==============================================
	int periodicTaskClass::start(int priority, int cpu){
		pthread_attr_t tattr;
		struct sched_param tparam;
		tparam.sched_priority=priority;
		
		pthread_attr_init(&tattr);//初始化线程属性
		pthread_attr_setschedpolicy(&tattr, SCHED_FIFO);//设置调度策略为：SCHED_FIFO
		pthread_attr_setschedparam(&tattr, &tparam);//设置优先级
		pthread_attr_setinheritsched(&tattr, PTHREAD_EXPLICIT_SCHED);//设置线程属性：不要继承 main 线程的调度策略和优先级。

		argStruct*arg=new argStruct({taskName,this,cpu});
		return pthread_create(&threadId, &tattr, loop, arg);
	}
	int periodicTaskClass::join(){
		return pthread_join(threadId,NULL);
	}
}//namespace

