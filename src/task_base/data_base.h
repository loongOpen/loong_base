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

对于不连续数据结构，继承baseStruct，手动处理数据结构
=====================================================*/
#pragma once
#include"eigen.h"

namespace Data{
using namespace std;

// 当数据结构不连续或存在浅拷贝风险，可继承此数据结构手动指定数据指针
// ①会被interface继承；②会被dc的shmBaseDataStruct继承，添加shm后再被dc继承
struct baseDataStruct{
	virtual int  getDataSize() final{return dataSizePrivate;}
	virtual void data2buf(char*buf) final;
	virtual void buf2data(char*buf) final;
	virtual vector<int>& getDataSizes()final{return sizes;}
	virtual vector<void*>& getDatas()final{return datas;}
protected:
	virtual int calSize(const int& mark)final;//mark可以随便写，仅用于输出报错，因此别重复
	vector<void*>datas;
	vector<int>sizes;
	int dataSizePrivate;
};
// ===============================================


}//namespace

