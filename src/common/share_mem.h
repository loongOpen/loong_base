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
共享内存
务必确保对接的数据结构是一块连续的内存空间！例如struct内包含vector不行！
=====================================================*/
#pragma once
#include<vector>
namespace Mem{
using namespace std;
// 在多进程or线程中多实例化，但根据creat参数可以获得同一片系统共享内存区域
// set会更新共享内存中的同一个心跳计数，get可获取各自的取用【负】计数，以便不同实例以各自的频率做掉线保护的判别
// 务必注意对接的数据结构的连续性！例如普通struct、或单vector都是连续的，但struct内包含vector就不连续！
class shareMemClass{
public:
	shareMemClass();
	~shareMemClass();
	int creat(int &mark,int size);//mark++返回，便于用户在序列中修改插入。
	void set(void*data);                                       //用于连续数据结构，将目标data写入到共享内存。
	void set(const vector<void*>datas, const vector<int>sizes);//用于不连续数据结构组，将一组目标data写入到共享内存。
	int  get(void*data, bool enforce=0);                                         //将共享内存取出到data。若后台数据未更新则不拷贝至data以提高性能(enforce强制拷贝)，返回shm数据最后一次更新后，get被调用的【负】次数
	int  get(const vector<void*>&datas, const vector<int>&sizes, bool enforce=0);//将共享内存取出到一组目标data。若后台数据未更新则不拷贝至data以提高性能(enforce强制拷贝)，返回shm数据最后一次更新后，get被调用的【负】次数
private:
	shareMemClass(const shareMemClass&)=delete;
	shareMemClass& operator=(const shareMemClass&)=delete;
	class impClass;
	impClass&imp;
};

}//namespace

