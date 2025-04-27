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

当数据结构不连续、或存在浅拷贝风险，可继承 Data::baseDataStruct 数据结构手动指定数据指针

=====================================================*/
#pragma once
#include"data_base.h"
namespace Task{
struct naviStruct{
	int checker;   //约定校验值
	int startTap;  //1踏步，0停止
	float vx,vy,wz,zOff;
};
struct cmdStruct{
	int head=99;
	int size;//32
	int id;
	int key=-1;
	float joy[4]{};
};
struct cctvCtrlStruct{
	short checker;
	short tgtGroup,tgtId; //全=0为广播
	short key;
};
struct cctvSensStruct{
	short id;
	short state;
};
struct jntSdkSensStruct:Data::baseDataStruct{
	int dataSize;			//数据帧c++散装内存大小
	double timestamp;		//epoch后秒数
	short key[2];			//遥控按键，当前所在plan之key
	char planName[16];		//当前所在plan之name
	short state[2];			//运行状态，任务状态
	float joy[4];			//遥控摇杆，可用于vx、vy、wz、zOff

	vec3f rpy,gyr,acc;		//imu
	vecXf actJ,actW,actT;	//实际关节角度、角速度、扭矩[左臂+右臂+颈+腰+左腿+右腿]
	vecXs drvTemp,drvState,drvErr;//驱动器温度、状态字、错误码[左臂+右臂+颈+腰+左腿+右腿]
	vecXf tgtJ,tgtW,tgtT;	//目标关节角度、角速度、扭矩[左臂+右臂+颈+腰+左腿+右腿]
	vecXf actFingerJ[2],tgtFingerJ[2];//手指角度[左+右]
	
	void init(int jntNum, int fingerDofLeft, int fingerDofRight){
		actJ.setZero(jntNum);
		actW.setZero(jntNum);
		actT.setZero(jntNum);
		drvTemp.setZero(jntNum);
		drvState.setZero(jntNum);
		drvErr.setZero(jntNum);
		tgtJ.setZero(jntNum);
		tgtW.setZero(jntNum);
		tgtT.setZero(jntNum);
		actFingerJ[0].setZero(fingerDofLeft);
		actFingerJ[1].setZero(fingerDofRight);
		tgtFingerJ[0].setZero(fingerDofLeft);
		tgtFingerJ[1].setZero(fingerDofRight);

		datas.emplace_back(&dataSize);
		datas.emplace_back(&timestamp);
		datas.emplace_back(key);
		datas.emplace_back(planName);
		datas.emplace_back(state);
		datas.emplace_back(joy);
		datas.emplace_back(rpy.data());
		datas.emplace_back(gyr.data());
		datas.emplace_back(acc.data());
		datas.emplace_back(actJ.data());
		datas.emplace_back(actW.data());
		datas.emplace_back(actT.data());
		datas.emplace_back(drvTemp.data());
		datas.emplace_back(drvState.data());
		datas.emplace_back(drvErr.data());
		datas.emplace_back(tgtJ.data());
		datas.emplace_back(tgtW.data());
		datas.emplace_back(tgtT.data());
		datas.emplace_back(actFingerJ[0].data());
		datas.emplace_back(actFingerJ[1].data());
		datas.emplace_back(tgtFingerJ[0].data());
		datas.emplace_back(tgtFingerJ[1].data());

		sizes.emplace_back(sizeof(dataSize));
		sizes.emplace_back(sizeof(timestamp));
		sizes.emplace_back(sizeof(key));
		sizes.emplace_back(sizeof(planName));
		sizes.emplace_back(sizeof(state));
		sizes.emplace_back(sizeof(joy));
		sizes.emplace_back(sizeof(rpy));
		sizes.emplace_back(sizeof(gyr));
		sizes.emplace_back(sizeof(acc));
		sizes.emplace_back(sizeof(actJ.value())*actJ.size());
		sizes.emplace_back(sizeof(actW.value())*actW.size());
		sizes.emplace_back(sizeof(actT.value())*actT.size());
		sizes.emplace_back(sizeof(drvTemp.value())*drvTemp.size());
		sizes.emplace_back(sizeof(drvState.value())*drvState.size());
		sizes.emplace_back(sizeof(drvErr.value())*drvErr.size());
		sizes.emplace_back(sizeof(tgtJ.value())*tgtJ.size());
		sizes.emplace_back(sizeof(tgtW.value())*tgtW.size());
		sizes.emplace_back(sizeof(tgtT.value())*tgtT.size());
		sizes.emplace_back(sizeof(actFingerJ[0].value())*actFingerJ[0].size());
		sizes.emplace_back(sizeof(actFingerJ[1].value())*actFingerJ[1].size());
		sizes.emplace_back(sizeof(tgtFingerJ[0].value())*tgtFingerJ[0].size());
		sizes.emplace_back(sizeof(tgtFingerJ[1].value())*tgtFingerJ[1].size());

		dataSize=calSize(54);
	}
};


//
#pragma pack(1)
struct ocuCmdStruct{
	unsigned int robotId;
	unsigned short bufferSize;
	unsigned char trlMode;//1
	float leftJoyLeft;//4
	float rightJoyRight;//4
	float leftJoyDown;//4
	int extraInt[8];//32
	float extraFloat[8];//32 //高度 速度 距离 角度 重心 LT RT 预留
	unsigned char gaitInfo[3];//3 gait_info[0]-0 ------ gait_info[1]-----cmd
	unsigned char extraByte[10];//10
};
#pragma pack()

#pragma pack(1)
struct ocuFeedbackStruct{
	float posExp[31];
	float posAct[31];
	float torExp[31];
	float torAct[31];
	float velExp[31];
	float velAct[31];
	float statusWord[31];
	float errorCode[31];
	float drvTemp[31];
	unsigned char sensorState[31];
	float rpy[3];
	float electricity;
	short gait;
	short gaitState;
	short ctrlMode;
	float vel;
	float posTarget[2];
	float posPerson[2];
	unsigned char flagMap;
	unsigned char imuState;
	unsigned char batteryState[4];
	unsigned char observe[4];
	short observeShort[4];
	float gps[3];
	float yuliu;
};
#pragma pack()
struct joyStruct{
	float leftUp, leftLeft, leftBack, rightUp, rightLeft, rightBack;
	void init(){
		leftUp=0; leftLeft=0; leftBack=0;
		rightUp=0; rightLeft=0; rightBack=0;
	}
	void clip(){
		Alg::clip(leftUp, 1);
		Alg::clip(leftLeft,1);
		Alg::clip(leftBack,1);
		Alg::clip(rightUp,1);
		Alg::clip(rightLeft,1);
		Alg::clip(rightBack,1);
	}
};
}//namespace

