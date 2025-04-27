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

#include"iopack.h"
#include"task_interface.h"
#include<unistd.h>
#include<signal.h>
#include<execinfo.h>

Task::interfaceTaskClass*intTask;

void sigHandle(int sig){
	print("\n== 终止 ==");
	if(intTask){
		intTask->stop();
	}
	usleep(10000);
	delete intTask;
}

void doTrace(int sig){
	const int size=200;
	void *buffer[size];
	char **strings;

	int nptrs=backtrace(buffer, size);
	strings=backtrace_symbols(buffer, nptrs);
	cout<<"===错误调用栈===\n";
	if(strings){
		for(int i=0;i<nptrs;++i){
			cout<<strings[i]<<endl;
		}
		free(strings);
	}
	cout<<"============\n";
	signal(sig,SIG_DFL);
}


int main(int argc, char* argv[]){
	if(getuid()!=0){throw runtime_error("权限不足");}
	signal(SIGINT,sigHandle);
	signal(SIGTERM,sigHandle);
	signal(SIGTSTP,sigHandle);
	signal(SIGSEGV, doTrace);

	Ini::iniClass iniThread("../config/thread.ini");
	int priority=iniThread.getVal("interfaceTask", "priority");
	int cpu=iniThread.getVal("interfaceTask", "cpu");
	float dt=iniThread.getVal("interfaceTask", "dt");

	intTask=new Task::interfaceTaskClass();
	intTask->init("udp",dt);
	intTask->start(priority, cpu);
	intTask->join();

	return 0;
}
