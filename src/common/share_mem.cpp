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

头指针pos +心跳 +数据块，数据分3块，由循环头指针索引，每次写一块，实现无锁结构
=====================================================*/
#include"share_mem.h"
#include<sys/shm.h>
#include<mutex>
#include<atomic>
#include"iopack.h"

namespace Mem{
class shareMemClass::impClass{
public:
	impClass();
	void creat(int mark, int size);
	void destroy();
	void set(void *data);
	void set(const vector<void*>&datas, const vector<int>&sizes);
	int  get(void *data,bool enforce);
	int  get(const vector<void*>&datas, const vector<int>&sizes,bool enforce);
	bool checkPtrErr();
	bool checkVecErr(const vector<void*>&datas, const vector<int>&sizes);
	int shmSize,dataSize;//shmSize=4+4+dataSize*3，头+心跳+数据块组
	
	char* memPtr=nullptr;
	int shmid=0;
	atomic<int> *pos,*heartBeat;//atomic指针指向共享内存中数据，可保证该部分也是原子而无需加锁
	mutex mut;//creat锁，不是数据块锁
	void*outerData;//绑定到外部的数据结构

	int heartBeatOld,getCnt;//get计数
};
	shareMemClass::impClass::impClass(){}
	void shareMemClass::impClass::creat(int mark,int size){
		mut.lock();
		if(memPtr){//防止同一实例多次creat
			print("共享内存不允许多次creat。mark=",mark);
			return;
		}
		getCnt=-10000;
		dataSize=size;
		shmSize=4 +4 +dataSize*3;//共享内存块大小：头指针 +心跳 +数据块

		key_t key=ftok("/home",mark);
		shmid=shmget(key,shmSize, 0666 | IPC_CREAT);
		if(-1==shmid){
			perror("shmget err");
			exit(-1);
		}
		memPtr=(char*)shmat(shmid, NULL, 0);
		if((char*)-1==memPtr){
			perror("shmat err");
			exit(-1);
		}
		struct shmid_ds buf;
		shmctl(shmid, IPC_STAT, &buf);
		if(buf.shm_nattch==1){
			memset(memPtr, 0, shmSize);
		}
		pos=(atomic<int>*)memPtr;
		heartBeat=(atomic<int>*)(memPtr+4);
		heartBeatOld=*heartBeat;
		mut.unlock();
	}
	void shareMemClass::impClass::destroy(){
		mut.lock();
		if(shmid){
			shmdt(memPtr);
			struct shmid_ds buf;
			shmctl(shmid, IPC_STAT, &buf);
			if(buf.shm_nattch==0){
				shmctl(shmid, IPC_RMID, 0);//标记删除状态
			}
			// shmctl(shmid, IPC_RMID, 0);//标记删除状态
		}
		mut.unlock();
		// cout<<"shm释放   ";
	}

	bool shareMemClass::impClass::checkPtrErr(){
		if(memPtr==nullptr){
			throw runtime_error("共享内存调用前未creat！");
			return 1;
		}
		return 0;
	}
	bool shareMemClass::impClass::checkVecErr(const vector<void*>&datas, const vector<int>&sizes){
		if(datas.size()!=sizes.size()){
			throw runtime_error("共享内存调用set vector与size数目不匹配！");
			return 1;
		}
		int totalSize=0;
		for(auto&it:sizes){
			totalSize+=it;
		}
		if(totalSize>dataSize){
			throw runtime_error("共享内存调用set vector size超过设定大小！");
			return 1;
		}
		return 0;
	}
	void shareMemClass::impClass::set(void*data){
		if(checkPtrErr()){return;}
		int tmp=*pos;
		tmp=(tmp+1)%3;
		memcpy(memPtr+8+dataSize*tmp,data,dataSize);
		(*pos)=tmp;
		(*heartBeat)++;//更新心跳
	}
	void shareMemClass::impClass::set(const vector<void*>&datas, const vector<int>&sizes){
		if(checkPtrErr()){return;}
		if(checkVecErr(datas,sizes)){return;}
		int tmp=*pos;
		tmp=(tmp+1)%3;
		char*head=memPtr+8+dataSize*tmp;
		For(datas.size()){
			memcpy(head,datas[i],sizes[i]);
			head+=sizes[i];
		}
		(*pos)=tmp;
		(*heartBeat)++;//更新心跳
	}
	int shareMemClass::impClass::get(void*data, bool enforce){
		checkPtrErr();
		int tmp=(*heartBeat);
		if(heartBeatOld !=tmp){
			memcpy(data,memPtr+8+dataSize*(*pos),dataSize);
			getCnt=1;
			heartBeatOld=tmp;
		}else if(enforce){
			memcpy(data,memPtr+8+dataSize*(*pos),dataSize);
		}
		if(getCnt>-1000000){
			getCnt--;
		}
		return getCnt;
	}
	int shareMemClass::impClass::get(const vector<void*>&datas, const vector<int>&sizes, bool enforce){
		checkPtrErr();
		checkVecErr(datas,sizes);
		int tmp=(*heartBeat);
		char*head=memPtr+8+dataSize*(*pos);
		if(heartBeatOld !=tmp){
			For(datas.size()){
				memcpy(datas[i],head,sizes[i]);
				head+=sizes[i];
			}
			getCnt=1;
			heartBeatOld=tmp;
		}else if(enforce){
			For(datas.size()){
				memcpy(datas[i],head,sizes[i]);
				head+=sizes[i];
			}
		}
		if(getCnt>-1000000){
			getCnt--;
		}
		return getCnt;
	}
// =============================================
	shareMemClass::shareMemClass():imp(*new impClass()){}
	shareMemClass::~shareMemClass(){
		imp.destroy();
		delete &imp;
	}
	int shareMemClass::creat(int &mark,int size){
		imp.creat(mark,size);
		mark++;
		return mark;//返回一个自增数，便于用户中间插入
	}
	// void shareMemClass::bind(void*data){imp.outerData=data;}
	// void shareMemClass::set(){imp.set(imp.outerData);}
	void shareMemClass::set(void*data){imp.set(data);}
	void shareMemClass::set(const vector<void*>datas, const vector<int>sizes){imp.set(datas,sizes);}
	
	// int  shareMemClass::get(bool enforce){return imp.get(imp.outerData, enforce);}
	int  shareMemClass::get(void*data, bool enforce){return imp.get(data, enforce);}
	int  shareMemClass::get(const vector<void*>&datas, const vector<int>&sizes, bool enforce){return imp.get(datas,sizes,enforce);}
	
}//namespace

