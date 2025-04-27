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
#include"task_interface.h"
#include"task_interface_data.h"
#include"udp.h"
#include<thread>
#include<chrono>
#include"timing.h"

namespace Task{
class interfaceTaskClass::impClass{
public:
	impClass();
	~impClass();
	void init(float dt);
	void update();
	void work();
	void dwdate();

	void udpRcv();
	void udpRcvOcu(char*buf);
	void udpRcvNavi(char*buf);
	void udpRcvApp(char*buf);
	void udpRcvJntSdk(char*buf);
	void udpRcvCctv(char*buf);
	float dt;
	
	cmdStruct cmd;
	cctvCtrlStruct cctvCtrl;
	cctvSensStruct cctvSens;
	naviStruct naviCmd;
	ocuCmdStruct ocuCmd;
	ocuFeedbackStruct ocuFeed;
	jntSdkSensStruct jntSdkSens;

	Data::dataCenterClass &dc=Data::dataCenterClass::instance();
	struct udpUnitStruct{
		Udp::udpServerClass server;
		string name;
		short checker;
		short lostCriteria;
		float lostCriteriaMs;
		bool lostFlag;
	};
	udpUnitStruct	udpOcu,		//遥控器
					udpNavi,	//导航
					udpApp,		//附加
					udpJntSdk,	//关节
					udpCctv;	//central control and telemetry vision 中控与遥测可视
	vector<udpUnitStruct*> udpUnits;

	short selfGroup,selfId;

	int keyOld=-1, cctvKeyOld=-1;
	bool naviFlag,cctvFlag;
	float vxMax,vyMax,wzMax,zOffMax,zOffMin;
	joyStruct joy;

	bool udpRunning;
	int rcvCnt,sendCnt;
	char sendBuf[1024*1024], recvBuf[1024*1024];
};
	interfaceTaskClass::impClass::impClass(){
		Ini::iniClass iniInterface("../config/interface.ini");
		selfGroup=iniInterface["selfGroup"];
		selfId=iniInterface["selfId"];
		cctvSens.id=selfId;
		vxMax=iniInterface["vxMax"];
		vyMax=iniInterface["vyMax"];
		wzMax=iniInterface["wzMax"];
		zOffMax=iniInterface["zOffMax"];
		zOffMin=iniInterface["zOffMin"];

		jntSdkSens.init(dc.drvNums, dc.fingerDofs[0], dc.fingerDofs[1]);

		udpUnits={&udpOcu, &udpNavi, &udpApp, &udpJntSdk, &udpCctv};
		vector<string> udpUnitNames{"ocu", "navi", "app", "jntSdk", "cctv"};
		For(udpUnits.size()){
			udpUnits[i]->name=udpUnitNames[i];
			udpUnits[i]->checker=iniInterface.getVal(udpUnitNames[i],"checker");
			udpUnits[i]->lostCriteriaMs=iniInterface.getVal(udpUnitNames[i],"lostCriteriaMs");//读进来时ms，需在init中根据dt转化为【负cnt】
			udpUnits[i]->lostFlag=1;
			bool en=iniInterface.getVal(udpUnitNames[i],"en");
			if(en){
				int port=iniInterface.getVal(udpUnitNames[i],"port");
				if(udpUnits[i]->server.openPort(port,1)){
					printL(udpUnitNames[i]," server port=",port);
				}
			}
		}
	}
	interfaceTaskClass::impClass::~impClass(){

	}
	void interfaceTaskClass::impClass::init(float dt){
		this->dt=dt;
		for(auto it:udpUnits){
			it->lostCriteria=-it->lostCriteriaMs/1000/dt;//读进来时ms，需在init中根据dt转化为【负cnt】
			it->lostFlag=1;
		}
		cctvKeyOld=-1;
		thread udpTh(&interfaceTaskClass::impClass::udpRcv,this);
		udpTh.detach();
	}
	void interfaceTaskClass::impClass::update(){
		dc.shmImu.get(&dc.imu);
		dc.shmAppOut.get(&dc.appOut);
		dc.shmLocoInfo.get(&dc.locoInfo);
		dc.jntCtrlLoco.getFromShm();
		dc.jntSensLoco.getFromShm();
		dc.fingerSens.getFromShm();
		// dc.fingerCtrl.getFromShm();
	}
	void interfaceTaskClass::impClass::work(){
		sendCnt++;
		if(dc.appOut.dataLen>0){
			udpApp.server.send(dc.appOut.data, dc.appOut.dataLen);
		}
		// ==ocu反馈
		if(sendCnt%10==0 && udpOcu.server.isConnect()){
			For(dc.drvNums){
				ocuFeed.posExp[dc.idMap[i]]=dc.jntCtrlLoco.j[i];
				ocuFeed.posAct[dc.idMap[i]]=dc.jntSensLoco.j[i];
				ocuFeed.torExp[dc.idMap[i]]=dc.jntCtrlLoco.t[i];
				ocuFeed.torAct[dc.idMap[i]]=dc.jntSensLoco.t[i];
				ocuFeed.velExp[dc.idMap[i]]=dc.jntCtrlLoco.w[i];
				ocuFeed.velAct[dc.idMap[i]]=dc.jntSensLoco.w[i];
				ocuFeed.statusWord[dc.idMap[i]]=dc.jntSensLoco.state[i];
				ocuFeed.errorCode[dc.idMap[i]]=dc.jntSensLoco.errCode[i];
				ocuFeed.drvTemp[dc.idMap[i]]=dc.jntSensLoco.temperature[i];
				ocuFeed.sensorState[dc.idMap[i]]=0;
			}
			For(4){
				ocuFeed.batteryState[i]=0;
				ocuFeed.observe[i]=0;
				ocuFeed.observeShort[i]=0;
			}
			For(3){
				ocuFeed.rpy[i]=dc.imu.rpy[i];
				ocuFeed.gps[i]=0.0;
			}
			ocuFeed.electricity=50;
			ocuFeed.gait=0;
			ocuFeed.gaitState=0;
			ocuFeed.ctrlMode=0;
			ocuFeed.vel=0.0;
			ocuFeed.posTarget[2]={0.0};
			ocuFeed.posPerson[2]={0.0};
			ocuFeed.flagMap=0;
			ocuFeed.imuState=0;
			ocuFeed.yuliu=0.0;
			udpOcu.server.send(&ocuFeed,sizeof(ocuFeed));
		}
		// ==中控反馈
		if(sendCnt%100==0 && udpCctv.server.isConnect()){
			cctvSens.state=cctvFlag | naviFlag<<1;
			udpCctv.server.send(&cctvSens, sizeof(cctvSensStruct));
			print(cctvSens.state);
		}
		// ==loco jnt sdk反馈
		if(udpJntSdk.server.isConnect()){
			jntSdkSens.timestamp=Timing::nowNs()/1e9;
			jntSdkSens.key[0]=dc.cmd.key;
			jntSdkSens.key[1]=dc.locoInfo.planKey;
			strcpy(jntSdkSens.planName, dc.locoInfo.planName);
			jntSdkSens.state[0]=dc.locoInfo.state[0];
			jntSdkSens.state[1]=dc.locoInfo.state[1];

			jntSdkSens.joy[0]=joy.leftUp;
			jntSdkSens.joy[1]=joy.leftLeft;
			jntSdkSens.joy[2]=joy.rightLeft;
			jntSdkSens.joy[3]=dc.cmd.zOff;

			jntSdkSens.rpy=dc.imu.rpy;
			jntSdkSens.gyr=dc.imu.gyr;
			jntSdkSens.acc=dc.imu.acc;
			
			jntSdkSens.actJ=dc.jntSensLoco.j;
			jntSdkSens.actW=dc.jntSensLoco.w;
			jntSdkSens.actT=dc.jntSensLoco.t;
			jntSdkSens.drvTemp=dc.jntSensLoco.temperature;
			jntSdkSens.drvState=dc.jntSensLoco.state;
			jntSdkSens.drvErr=dc.jntSensLoco.errCode;

			jntSdkSens.tgtJ=dc.jntCtrlLoco.j;
			jntSdkSens.tgtW=dc.jntCtrlLoco.w;
			jntSdkSens.tgtT=dc.jntCtrlLoco.t;

			jntSdkSens.actFingerJ[0]=dc.fingerSens.j[0];
			jntSdkSens.actFingerJ[1]=dc.fingerSens.j[1];
			// jntSdkSens.tgtFingerJ<<dc.fingerCtrl.j[0], dc.fingerCtrl.j[1];

			//测试
			// jntSdkSens.key[0]=55;
			// jntSdkSens.planName[3]='p';
			// jntSdkSens.state[1]=5;
			// jntSdkSens.rpy<<7,5,4;
			// jntSdkSens.gyr<<-1,2,3;
			// jntSdkSens.gyr<<0,2,9;
			// jntSdkSens.actT<<0,0,0,5,6,1;
			// jntSdkSens.tgtT<<0,0,0,5,6,1;
			// jntSdkSens.drvTemp<<45,6,0,0,78;
			// jntSdkSens.tgtFingerJ<<6,7,5,0,0,0,0,3,4;

			jntSdkSens.data2buf(sendBuf);
			udpJntSdk.server.send(sendBuf, jntSdkSens.getDataSize());
		}
	}
	void interfaceTaskClass::impClass::dwdate(){

	}
// =============
	void interfaceTaskClass::impClass::udpRcv(){
		Timing::sleepMs(20);
		bool udpRunning=1;
		auto refClock=chrono::high_resolution_clock().now();
		int sleepUs=this->dt*1e6;
		while(udpRunning){
			udpRcvOcu(recvBuf);//遥控
			udpRcvCctv(recvBuf);
			udpRcvNavi(recvBuf);//导航
			udpRcvJntSdk(recvBuf);
			udpRcvApp(recvBuf);//附加通信
			rcvCnt++;
			refClock+=chrono::microseconds(sleepUs);
			this_thread::sleep_until(refClock);
		}
	}

	//遥控器
	void interfaceTaskClass::impClass::udpRcvOcu(char*buf){
		static bool firstFlag=1;
		int len=udpOcu.server.recv(buf);
		if(len>0){
			ocuCmdStruct &tmp=*(ocuCmdStruct*)buf;
			int key=tmp.gaitInfo[1];
			// 死区保护
			joy.leftUp   =-Alg::threshed(tmp.leftJoyDown,20)/80;
			joy.leftLeft = Alg::threshed(tmp.leftJoyLeft,20)/80;
			joy.leftBack = Alg::threshed(tmp.extraFloat[5],20)/80;
			joy.rightUp  =-Alg::threshed(tmp.extraFloat[4],20)/80;
			joy.rightLeft=-Alg::threshed(tmp.rightJoyRight,20)/80;
			joy.rightBack = Alg::threshed(tmp.extraFloat[6],20)/80;
			joy.clip();
			
			if(udpOcu.lostFlag){//第一帧拿firstKey
				keyOld=key;
				udpOcu.lostFlag=0;
				firstFlag=1;
				print("ocu 连接 ！");
			}

			if(firstFlag){
				if(keyOld!=key){//第一帧必相等而跳过；等第一次改变后进入
					firstFlag=0;
					dc.cmd.key=key;
				}
			}else if(key !=keyOld){
				switch(key){
				case 200:
					cctvFlag=0;
					print("中控介出");
					break;
				case 201:
					cctvFlag=1;
					print("中控介入");
					break;
				case 204:
					naviFlag=0;
					dc.cmd.naviTap=-1;
					print("导航介出");
					break;
				case 205:
					naviFlag=1;
					print("导航介入");
					break;
				}
				dc.cmd.key=key;
			}
			// 是否cctv覆盖。综合效果应该是：ocu或cctv谁改变，就dc.cmd.key=谁。
			if(cctvFlag && (!udpCctv.lostFlag)){
				if(cctvKeyOld>-1 && cctvCtrl.key!=cctvKeyOld){
					dc.cmd.key=cctvCtrl.key;
					printL("cctv=",dc.cmd.key);
				}
				cctvKeyOld=cctvCtrl.key;
			}else{
				cctvKeyOld=-1;
			}
			// 是否导航覆盖
			if(naviFlag && joy.leftUp==0 && joy.leftLeft==0 && joy.rightLeft==0){
				dc.cmd.vx=naviCmd.vx;
				dc.cmd.vy=naviCmd.vy;
				dc.cmd.wz=naviCmd.wz;
				dc.cmd.naviTap=naviCmd.startTap;
			}else{
				dc.cmd.vx=joy.leftUp *vxMax;
				dc.cmd.vy=joy.leftLeft *vyMax;
				dc.cmd.wz=joy.rightLeft *wzMax;
			}
			// zOff单独控
			if(naviFlag){
				dc.cmd.zOff=naviCmd.zOff;
			}else{
				dc.cmd.zOff+=(joy.leftBack -joy.rightBack)*1e-2;
			}
			Alg::clip(dc.cmd.vx, vxMax);
			Alg::clip(dc.cmd.vy, vyMax);
			Alg::clip(dc.cmd.wz, wzMax);
			Alg::clip(dc.cmd.zOff, zOffMin, zOffMax);

			if(key !=keyOld){
				printf("key=%d vx=%f vy=%f wz=%f zOff=%f navi=%d\n",
						key, dc.cmd.vx,dc.cmd.vy, dc.cmd.wz, dc.cmd.zOff, dc.cmd.naviTap);
				fflush(stdout);
			}
			keyOld=key;
			dc.shmCmd.set(&dc.cmd);
		}else if(len<udpOcu.lostCriteria){
			if(!udpOcu.lostFlag){
				udpOcu.lostFlag=1;
				dc.cmd.key=-1;
				dc.cmd.vx=0;
				dc.cmd.vy=0;
				dc.cmd.wz=0;
				// dc.shmCmd.set(&dc.cmd);//其他进程自动判断不必set
				print("ocu 离线！");
				udpOcu.server.allowNew();
			}
		}
		if(rcvCnt%1000==11){
			if(udpOcu.lostFlag){
				print("ocu 离线！");
			}
			printf("key=%d vx=%f vy=%f wz=%f zOff=%f navi=%d\n",
					dc.cmd.key, dc.cmd.vx, dc.cmd.vy, dc.cmd.wz, dc.cmd.zOff, dc.cmd.naviTap);
			fflush(stdout);
		}
	}
	//中控
	void interfaceTaskClass::impClass::udpRcvCctv(char*buf){
		int len=udpCctv.server.recv(buf);
		bool checked=1;
		if(len>0){
			if(((cctvCtrlStruct*)buf)->checker!=udpCctv.checker){
				checked=0;
			}
			if(checked){
				if(udpCctv.lostFlag){
					print("cctv 连接！");
					udpCctv.lostFlag=0;
				}
				short group=(*(cctvCtrlStruct*)buf).tgtGroup;
				short id=(*(cctvCtrlStruct*)buf).tgtId;
				if((group|id)==0 || group==selfGroup || id==selfId){//group、id全=0为广播，或匹配其中任一
					cctvCtrl=*(cctvCtrlStruct*)buf;
				}
			}
		}else if(len<udpCctv.lostCriteria){
			if(!udpCctv.lostFlag){
				print("cctv 离线...");
				udpCctv.lostFlag=1;
			}
		}
	}
	//导航
	void interfaceTaskClass::impClass::udpRcvNavi(char*buf){
		int len=udpNavi.server.recv(buf);
		bool checked=1;
		if(len>0){
			if(((naviStruct*)buf)->checker!=udpNavi.checker){
				checked=0;
			}
			if(checked){
				if(udpNavi.lostFlag){
					print("navi 连接！");
					udpNavi.lostFlag=0;
				}
				naviCmd=*(naviStruct*)buf;
			}
		}else if(len<udpNavi.lostCriteria){
			if(!udpNavi.lostFlag){
				print("navi 离线");
				udpNavi.lostFlag=1;
				naviCmd.vx=0;
				naviCmd.vy=0;
				naviCmd.wz=0;
				udpNavi.server.allowNew();
			}
		}
		if(rcvCnt%1000==8){
			if(!udpNavi.lostFlag){
				printf("navi tap=%d vx=%f vy=%f wz=%f zOff=%f",
						naviCmd.startTap, naviCmd.vx, naviCmd.vy, naviCmd.wz, naviCmd.zOff);
			}
			if(!checked){
				printf("navi checker 不匹配！");
			}
		}
	}
	//joint sdk
	void interfaceTaskClass::impClass::udpRcvJntSdk(char*buf){
		int len=udpJntSdk.server.recv(buf);
		short checker=*(short*)(buf);
		short size=*(short*)(buf+2);
		if(len==dc.locoJntSdk.getDataSize() && len==size){
			if(checker!=udpJntSdk.checker){
				print("jnt sdk udp checker不匹配!");
				return;
			}
			if(udpJntSdk.lostFlag){
				udpJntSdk.lostFlag=0;
				print("jnt sdk 连接！");
			}
			dc.locoJntSdk.setToShmDirectly(buf);
		}else if(len>0){
			printL("jnt sdk udp接收size=",len,"，不匹配目标size=",dc.locoJntSdk.getDataSize());
		}else if(len<udpJntSdk.lostCriteria){
			if(udpJntSdk.lostFlag==0){
				udpJntSdk.lostFlag=1;
				print("jnt sdk 离线");
				udpJntSdk.server.allowNew();
			}
		}
	}
	//附加
	void interfaceTaskClass::impClass::udpRcvApp(char*buf){
		int len=udpApp.server.recv(buf);
		if(len>0 && len<256){
			dc.appIn.dataLen=len;
			memcpy(dc.appIn.data, buf, len);
			dc.shmAppIn.set(&dc.appIn);
		}else if(len<udpApp.lostCriteria){
			if(udpApp.lostFlag==0){
				udpApp.lostFlag=1;
				print("udp app 离线");
				udpApp.server.allowNew();
			}
		}
	}
//=================================================================
	interfaceTaskClass::interfaceTaskClass():imp(*new impClass()){

	}
	interfaceTaskClass::~interfaceTaskClass(){
		imp.udpRunning=0;
		delete &imp;
	}
	void interfaceTaskClass::init(const string &name,float dt){
		taskName=name;
		this->dt=dt;
		imp.init(dt);
	}
	bool interfaceTaskClass::step(){
		imp.update();
		imp.work();
		imp.dwdate();
		return running;
	}

}//namespace

