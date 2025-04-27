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
#include"data_base.h"
#include<sstream>

namespace Data{
using namespace std;
	void baseDataStruct::data2buf(char*buf){
		int idx=0;
		For(datas.size()){
			memcpy(buf+idx, datas[i], sizes[i]);
			idx+=sizes[i];
		}
	}
	void baseDataStruct::buf2data(char*buf){
		int idx=0;
		For(datas.size()){
			memcpy(datas[i], buf+idx, sizes[i]);
			idx+=sizes[i];
		}
	}
	int baseDataStruct::calSize(const int& mark){//mark可以随便写，仅用于输出报错，因此别重复
		if(datas.size()!=sizes.size()){
			stringstream ss;
			ss<<"某继承类的datas.size != sizes.size！调用mark="<<mark;
			throw runtime_error(ss.str());
		}
		dataSizePrivate=0;
		For(sizes.size()){
			dataSizePrivate+=sizes[i];
		}
		return dataSizePrivate;
	}


}//namespace

