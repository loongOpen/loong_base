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

#include"nabo_data.h"
=====================================================*/
#pragma once
#include"eigen.h"
namespace Nabo{
using namespace std;

//==传入组=========================================
struct cmdStruct{
	int key=-1;			//-1=掉线，0=未更新，>0=按键值
	float vx,vy,wz,zOff;
	int naviTap=-1;		//-1：不导航，0：导航不踏步，1：导航踏步
};
struct maniSdkStruct{
	short inCharge=1;	//当loco、mani同时运行，mani是否介入控制
	short filtLevel;	//滤波等级0~5，0最大滤波，>=5不滤波
	
	short armMode;		//0无，1回正，2下肢命令透传，3关节轴控，4笛卡尔身体系
	short fingerMode;	//0无，1回正，2下肢命令透传，3关节轴控，4伸直
	short neckMode;		//0无，1回正，2下肢命令透传，3关节轴控，4导航随动，5看左手，6看右手，7看双手中间
	short lumbarMode;	//0无，1回正，2下肢命令透传，3关节轴控，4姿态控制

	vecXf armCmd[2];	//armMode=2: xyz+rpy+臂型角。armMode=3: 轴控。至少6维，至多armDof
	vec6f armFM[2];		//前馈，3力+3扭，armMode=2、3时生效
	vecXf fingerCmd[2]; //fingerDofs
	vecXf neckCmd;		//neckDof
	vecXf lumbarCmd;	//lumbarDof
	void init(int armDof,int fingerDofLeft,int fingerDofRight,int neckDof,int lumbarDof){//在驱动底层调用
		armCmd[0].setZero(max(6,armDof));
		armCmd[1].setZero(max(6,armDof));
		armCmd[0]<<0.2, 0.2,-0.1;
		armCmd[1]<<0.2,-0.2,-0.1;
		fingerCmd[0].setZero(fingerDofLeft);
		fingerCmd[1].setZero(fingerDofRight);
		neckCmd.setZero(neckDof);
		lumbarCmd.setZero(lumbarDof);
	}
};
struct sensorStruct{
	int cnt;
	vec3f rpy,gyr,acc;
	vec3f supP,supV;
	vecXf j,w,t;
	vecXs state;
	vecXf finger[2];
	void init(int jntNum, int fingerDofLeft, int fingerDofRight){//在驱动底层调用
		j.setZero(jntNum);
		w.setZero(jntNum);
		t.setZero(jntNum);
		state.setZero(jntNum);
		finger[0].setZero(fingerDofLeft);
		finger[1].setZero(fingerDofRight);
	}
};
struct jntSkdStruct{
	int state;
	float torLimitRate=0.1;//关节输出最大电流限制[0~1]
	float filtRate=0.1;//滤波系数[0~1]，不是截止频率！输出=filtRate*输入 +(1-filtRate)*上一次输出
	vecXf j,w,t,kp,kd;
	vecXf finger[2];
	void init(int jntNum, int fingerDofLeft, int fingerDofRight){//在驱动底层调用
		j.setZero(jntNum);
		w.setZero(jntNum);
		t.setZero(jntNum);
		kp.setZero(jntNum);
		kd.setZero(jntNum);
		finger[0].setZero(fingerDofLeft);
		finger[1].setZero(fingerDofRight);
	}
};
//==传出组=========================================
struct ctrlStruct{
	bool enFlag=0;
	vecXf j,w,t;
	vecXf finger[2];
	void init(int jntNum, int fingerDofLeft, int fingerDofRight){//在驱动底层调用
		j.setZero(jntNum);
		w.setZero(jntNum);
		t.setZero(jntNum);
		finger[0].setZero(fingerDofLeft);
		finger[1].setZero(fingerDofRight);
	}
};
struct paramStruct{
	vecXf kp,kd,maxTor;
	void init(int jntNum){
		kp.setZero(jntNum);
		kd.setZero(jntNum);
		maxTor.setZero(jntNum);
	}
};
struct infoStruct{
	int planKey;
	char planName[16];
	short state[2];
	// bodyP于世界系，其余均为操作系F（操作系F始终以机身为原点，始终以当前front朝向为x，z竖直向上）
	vec3f bodyP,bodyV2F;
	vec6f tipPRpy2F[2],tipVW2F[2],tipFM2F[2];
};
//==透传结构========================================
struct appStruct{
	// data不是每个plan都会用，自定义数据流，直接透传，用户自己解析。data字段务必添加特有验证信息，以免不同用户误用
	int dataLen=0;
	char data[256]{};
};






//==最终封装=========================================
struct inputStruct{
	cmdStruct cmd;
	maniSdkStruct maniSdk;
	sensorStruct sens;
	jntSkdStruct jntSdk;
	appStruct app;//app data不是每个plan都会用，自定义数据流，直接透传，用户自己解析。data字段务必添加特有验证信息，以免不同用户误用
};
struct outputStruct{
	ctrlStruct ctrl;
	paramStruct param;
	infoStruct info;
	appStruct app;//app data不是每个plan都会用，自定义数据流，直接透传，用户自己解析。data字段务必添加特有验证信息，以免不同用户误用
};

}//namespace
