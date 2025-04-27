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
共享内存数据中心：
在不同线程获取不同实例、以及相对应的成员，
成员状态计数互相独立，但所需的交互的数据指向同一片共享内存。

共享数据的两种用法：
  对于不连续数据结构：继承baseStruct，内含共享内存实例，通过vector手动处理不连续
  对于连续数据结构：平行定义数据类和相应共享内存
=====================================================*/
#pragma once
#include"share_mem.h"
#include"eigen.h"
#include"loong_driver_sdk/loong_driver_sdk.h"
#include"data_base.h"

namespace Data{
using namespace std;

// 继承baseStruct的数据能力，添加共享内存支持
struct shmBaseDataStruct:baseDataStruct{
	virtual void setToShmDirectly(void*data)final{shm.set(data);};//不清楚数据结构就不要用！外部数据data一步到位
	virtual int  getFromShmDirectly(void*data, bool enforce=0)final{return shm.get(data,enforce);};//不清楚数据结构就不要用！外部数据data一步到位
	virtual void setToShm()final{shm.set(datas,sizes);};
	virtual int  getFromShm(bool enforce=0)final{return shm.get(datas,sizes,enforce);}
protected:
	Mem::shareMemClass shm;
	virtual int shmCreat(int &mark)final;
};

// ==存在浅拷贝风险、不连续数据结构，需调用shm的vector相关接口================
struct jntSensStruct:shmBaseDataStruct{
	vecXf j,w,t;
	vecXs state,errCode,temperature;
	// finger的sense单列出来
	int init(int &mark, int jntNum);
};
struct jntCtrlStruct:shmBaseDataStruct{
	vecXf j,w,t,kp,kd,maxTor;
	vecXs enable;
	bool inCharge;
	vecXf fingerJ[2];
	int init(int &mark, int jntNum, int fingerDofLeft, int fingerDofRight);
};
struct locoJntSdkStruct:shmBaseDataStruct{
	short checker;			//约定值（interface.ini内定义）
	short size;				//c++散装内存大小
	int state;				//0不执行，
	float torLimitRate=0.1;	//关节输出最大电流限制[0~1]
	float filtRate=0.05;	//滤波系数[0~1]，不是截止频率！输出=filtRate*输入 +(1-filtRate)*上一次输出
	vecXf j,w,t,kp,kd;
	vecXf finger[2];
	int init(int &mark, int jntNum, int fingerDofLeft, int fingerDofRight);
};

struct fingerSensStruct:shmBaseDataStruct{//双手区分
	vecXf j[2],w[2];
	int init(int &mark, int fingerDofLeft, int fingerDofRight);
};

// ==普通数据结构，直接给shm传指针===============================
struct imuStruct{
	vec3f rpy,gyr,acc;
};
struct cmdStruct{
	int key;
	float vx,vy,wz,zOff;
	int naviTap=-1;
};
struct appStruct{
	// data不是每个plan都会用，自定义数据流，直接透传，用户自己解析。data字段务必添加特有验证信息，以免不同用户误用
	int dataLen=0;
	char data[256]{};
};

struct locoInfoStruct{
	int planKey;		//当前所在plan之key
	char planName[16];	//当前所在plan之name
	short state[2];		//运行状态，任务状态
	vec3f bodyP,bodyV2F;
	vec6f tipPRpy2F[2],tipVW2F[2],tipFM2F[2];
};


//=========================================
class dataCenterClass{
public:
	static dataCenterClass& instance();
	int drvNums;
	int maniDrvNums;
	vector<int> idMap;//硬件和pino顺序不对应，pino[i]=硬件[idmap[i]]
	int armDof;//单臂
	int fingerDofs[2], fingerDofBoth;//双手构型可能不同, fingerDofBoth=fingerDofs[0]+fingerDofs[1]
	int lumbarDof;
	int neckDof;
	int legDof;

	// 不连续数据结构，struct内部自定义shm分段接口，调用自身shm相关函数即可
	jntSensStruct jntSensLoco;
	jntCtrlStruct jntCtrlLoco;
	locoJntSdkStruct locoJntSdk;
	fingerSensStruct fingerSens;

	// 连续数据结构，shm直接搭配即可
	Mem::shareMemClass  shmImu,
						shmCmd,
						shmAppIn,
						shmAppOut,
						shmLocoOpenFlag,
						shmManiInfo,
						shmLocoInfo;
	imuStruct imu;
	cmdStruct cmd;
	appStruct appIn,appOut;
	locoInfoStruct locoInfo;

	int locoOpenFlag;
private:
	dataCenterClass();
	dataCenterClass(const dataCenterClass&)=delete;
	dataCenterClass & operator=(const dataCenterClass&)=delete;
};

}//namespace

