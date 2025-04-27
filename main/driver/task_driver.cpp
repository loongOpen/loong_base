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
#include"task_driver.h"
#include"loong_driver_sdk/loong_driver_sdk.h"
#include"loong_driver_sdk/joint_map.h"
#include"log.h"
#include<unistd.h>

namespace Task{
	
static const float CntBase=2000;



class driverTaskClass::impClass{
public:
	impClass();
	~impClass(){
		print("drv imp 释放");
	};
	void init(float dt);
	void update();
	void work();
	void dwdate();
	void log();
	void enforceDisable();
	float dt;
	int cnt=0;
	int waitOP,activeDrvNum;
	float maxCurRate;
	string sdkXml;
	int sdkCpuId;
	float imuAdj[2];

	Data::dataCenterClass &dc=Data::dataCenterClass::instance();

	vector<DriverSDK::motorActualStruct> motActs;
	vector<DriverSDK::motorTargetStruct> motTgts;
	vector<DriverSDK::motorActualStruct*> motActPtrMaped;
	vector<DriverSDK::motorTargetStruct*> motTgtPtrMaped;
	vector<DriverSDK::digitActualStruct> fingerActs;
	vector<DriverSDK::digitTargetStruct> fingerTgts;
	vector<int> motBiasCnt;
	vector<int*> motBiasCntMaped;
	vector<unsigned short> motMaxCurs;
	struct{//中间值
		vecXf j,w,t,kp,kd,maxTor,fingerJ[2];
		vecXs en;
		void init(int jntNums, int fingerDofLeft, int fingerDofRight){
			j.resize(jntNums);
			w.resize(jntNums);
			t.resize(jntNums);
			kp.resize(jntNums);
			kd.resize(jntNums);
			maxTor.resize(jntNums);
			en.resize(jntNums);
			fingerJ[0].setZero(fingerDofLeft);
			fingerJ[1].setZero(fingerDofRight);
		}
	}jntCtrl;
	struct{
		vecXf pos,vel,tor;
		void init(int jntNums){
			pos.resize(jntNums);
			vel.resize(jntNums);
			tor.resize(jntNums);
		}
	}motCtrl;
	struct{
		vecXf kpMax,kdMax,maxPos,minPos,maxVel,maxTor,cutFrq;
		void init(int jntNums){
			kpMax.resize(jntNums);
			kdMax.resize(jntNums);
			maxPos.resize(jntNums);
			minPos.resize(jntNums);
			maxVel.resize(jntNums);
			maxTor.resize(jntNums);
			cutFrq.resize(jntNums);
		}
	}jntLims;
	vector<Alg::filterOneClass> jntFil;

	DriverSDK::imuStruct sdkImu;
	DriverSDK::DriverSDK &sdk=DriverSDK::DriverSDK::instance();

	static const int limbNum=5;
	array<int,limbNum> limbGroup;
	string limbName[limbNum]{"l arm", "r arm", "neck+lumbar", "l leg", "r leg"};

	enum State:int{
		Never=0,Online=1,Lost=2,
	};
	State locoState=Never,maniState=Never;

	Log::logClass logger;
	stringstream logSS;
	int logCnt;
};

	driverTaskClass::impClass::impClass(){
		limbGroup={dc.armDof, dc.armDof, dc.neckDof+dc.lumbarDof, dc.legDof, dc.legDof};
		jntFil.resize(dc.drvNums);

		jntLims.init(dc.drvNums);
		jntCtrl.init(dc.drvNums, dc.fingerDofs[0], dc.fingerDofs[1]);
		motCtrl.init(dc.drvNums);

		motActs.resize(dc.drvNums);
		motTgts.resize(dc.drvNums);
		motBiasCnt.resize(dc.drvNums);
		motActPtrMaped.resize(dc.drvNums);
		motTgtPtrMaped.resize(dc.drvNums);
		motBiasCntMaped.resize(dc.drvNums);
		motMaxCurs.resize(dc.drvNums);
		vector<int> maxCursMaped;
		maxCursMaped.resize(dc.drvNums);

		fingerActs.resize(dc.fingerDofs[0] +dc.fingerDofs[1]);
		fingerTgts.resize(dc.fingerDofs[0] +dc.fingerDofs[1]);
		
		Ini::iniClass iniDrv("../config/driver.ini");
		
		iniDrv.getArray("imuAdj",imuAdj,2);

		iniDrv.getArray("joint","maxCur",maxCursMaped.data(), dc.drvNums);
		iniDrv.getArray("joint","motBiasCnt",motBiasCnt.data(), dc.drvNums);
		iniDrv.getArray("joint","kpMax",jntLims.kpMax.data(), dc.drvNums);
		iniDrv.getArray("joint","kdMax",jntLims.kdMax.data(), dc.drvNums);
		iniDrv.getArray("joint","maxPos",jntLims.maxPos.data(), dc.drvNums);
		iniDrv.getArray("joint","minPos",jntLims.minPos.data(), dc.drvNums);
		iniDrv.getArray("joint","maxVel",jntLims.maxVel.data(), dc.drvNums);
		iniDrv.getArray("joint","maxTor",jntLims.maxTor.data(), dc.drvNums);
		iniDrv.getArray("joint","cutFrq",jntLims.cutFrq.data(), dc.drvNums);
		maxCurRate=iniDrv.getVal("joint","maxCurRate");

		sdkXml="../config/"+iniDrv.getStr("sdkXml");
		logger.init("drive");
		logCnt=iniDrv["logCnt"];


		Ini::iniClass iniThread("../config/thread.ini");
		sdkCpuId=iniThread.getVal("driverSdk", "cpu");

		For(dc.drvNums){
			motActPtrMaped[i]=&motActs[dc.idMap[i]];
			motTgtPtrMaped[i]=&motTgts[dc.idMap[i]];
			motBiasCntMaped[i]=&motBiasCnt[dc.idMap[i]];
			motMaxCurs[dc.idMap[i]]=maxCursMaped[i] *maxCurRate;
		}
	}
	void driverTaskClass::impClass::init(float dt){
	#ifdef DefSim
		waitOP=0;
	#else
		waitOP=100;
		if(access(sdkXml.data(), 0)==-1){
			stringstream ss;
			ss<<"未找到"<<sdkXml;
			throw runtime_error(ss.str());
		}
	#endif
		this->dt=dt;
		
		For(dc.drvNums){
			jntFil[i].init(dt,jntLims.cutFrq[i]);
		}
		sdk.setMaxCurr(motMaxCurs);
		sdk.setCPU(sdkCpuId);
		sdk.init(sdkXml.data());
		// sdk.setCntBias();
		if(dc.drvNums!=sdk.getTotalMotorNr()){
			printL("driver.ini驱动数=",dc.drvNums,",sdk xml电机数=",sdk.getTotalMotorNr());
			throw runtime_error("驱动数目不匹配！");
		}

		auto activeMots=sdk.getActiveMotors();
		activeDrvNum=activeMots.size();
	}
	void driverTaskClass::impClass::update(){
		cnt++;
		//==关节传感采集
		
		sdk.getMotorActual(motActs);

		int id=0;
		if(cnt%2000==0){//不集中打印，以防影响实时性
			cout<<cnt<<endl;
			cout<<">>上传 mot pos [l arm |r arm |neck+lumbar |l leg |r leg]:\n";
			For(limbNum){
				for(int g=0;g<limbGroup[i];g++){
					cout<<motActPtrMaped[id+g]->pos<<", ";
				}
				id+=limbGroup[i];
				if(limbGroup[i]){cout<<endl;}
			}
		}

		if(waitOP){
			int tmp=0;
			For(dc.drvNums){
				if(short(motActPtrMaped[i]->statusWord)>0)//未激活=-1，未OP=0
				tmp++;
			}
			if(tmp==activeDrvNum){
				waitOP--;
			}
			if(cnt%2000==20){//不集中打印，以防影响实时性
				printL("OP驱动数=",tmp);
			}
			return;
		}

		For(dc.drvNums){
			dc.jntSensLoco.j[i]=motActPtrMaped[i]->pos;
			dc.jntSensLoco.w[i]=motActPtrMaped[i]->vel;
			dc.jntSensLoco.t[i]=motActPtrMaped[i]->tor;
			dc.jntSensLoco.state[i]=motActPtrMaped[i]->statusWord;
			dc.jntSensLoco.errCode[i]=motActPtrMaped[i]->errorCode;
			dc.jntSensLoco.temperature[i]=motActPtrMaped[i]->temp;
		}
	#ifndef DefSim
		JntMap::update(dc.jntSensLoco.j, dc.jntSensLoco.w, dc.jntSensLoco.t);
	#endif
		dc.jntSensLoco.setToShm();

		//==imu采集
		sdk.getIMU(sdkImu);
		For(3){
			dc.imu.rpy[i]=sdkImu.rpy[i];
			dc.imu.gyr[i]=sdkImu.gyr[i];
			dc.imu.acc[i]=sdkImu.acc[i];
		}
		dc.imu.rpy[0]+=imuAdj[0];
		dc.imu.rpy[1]+=imuAdj[1];
		dc.shmImu.set(&dc.imu);
		if(cnt%2000==1){
			cout<<">>imu rpy =";
			For3{
				cout<<sdkImu.rpy[i]<<", ";
			}
			cout<<endl;
		}
		if(abs(sdkImu.rpy[0])>2 || abs(sdkImu.rpy[1])>2){
			print("imu rpy异常", sdkImu.rpy[0], sdkImu.rpy[1], sdkImu.rpy[2]);
		}
		if(hypot(sdkImu.gyr[0], sdkImu.gyr[1], sdkImu.gyr[2])>50){
			print("imu gyr异常", sdkImu.gyr[0], sdkImu.gyr[1], sdkImu.gyr[2]);
		}
		//==手指
		sdk.getDigitActual(fingerActs);
		For(dc.fingerDofs[0]){
			dc.fingerSens.j[0][i]=fingerActs[i].pos;
		}
		For(dc.fingerDofs[1]){
			dc.fingerSens.j[1][i]=fingerActs[i+dc.fingerDofs[0]].pos;
		}
		dc.fingerSens.setToShm();

		//==遥控器命令
		// dc.shmCmd.get(&dc.cmd);
		// switch(dc.cmd.key){
		// 	break;
		// }
	}
	void driverTaskClass::impClass::work(){
		int id;
		if(cnt%2000==2){//不集中打印，以防影响实时性
			// cout<<cnt<<endl;
			bool err[limbNum]{};
			id=0;
			cout<<">>状态字 [l arm |r arm |neck+lumbar |l leg |r leg]:\n";
			For(limbNum){
				for(int g=0;g<limbGroup[i];g++){
					cout<<dc.jntSensLoco.state[id+g]<<", ";
					err[i]|=dc.jntSensLoco.errCode[id+g];
				}
				id+=limbGroup[i];
				if(limbGroup[i]){cout<<endl;}
			}
			id=0;
			For(limbNum){
				if(err[i]){
					cout<<limbName[i]<<" errCode(hex): ";
					for(int g=0;g<limbGroup[i];g++){
						printf("%X, ",dc.jntSensLoco.errCode[id+g]);
					}
					cout<<endl;
				}
				id+=limbGroup[i];
			}
		}
		if(cnt%2000==3){//不集中打印，以防影响实时性
			id=0;
			cout<<">>上传 jnt pos [l arm |r arm |neck+lumbar |l leg |r leg]:\n";
			For(limbNum){
				for(int g=0;g<limbGroup[i];g++){
					cout<<dc.jntSensLoco.j[id+g]<<", ";
				}
				id+=limbGroup[i];
				if(limbGroup[i]){cout<<endl;}
			}
		}
		if(cnt%5000==7){//不集中打印，以防影响实时性
			if(locoState!=Online){"loco离线";}
			if(maniState!=Online){"mani离线";}
		}
	}
	void driverTaskClass::impClass::dwdate(){
		if(waitOP==0){
			if(dc.jntCtrlLoco.getFromShm()>-100){
				if(locoState!=Online){
					print("==loco上线！==");
					locoState=Online;
				}
			}else{
				if(locoState==Online){
					print("==loco离线！==");
					locoState=Lost;
				}
			}
			jntCtrl.j=dc.jntCtrlLoco.j;
			jntCtrl.w=dc.jntCtrlLoco.w;
			jntCtrl.t=dc.jntCtrlLoco.t;
			jntCtrl.fingerJ[0]=dc.jntCtrlLoco.fingerJ[0];
			jntCtrl.fingerJ[0]=dc.jntCtrlLoco.fingerJ[0];
			jntCtrl.kp=dc.jntCtrlLoco.kp;
			jntCtrl.kd=dc.jntCtrlLoco.kd;
			jntCtrl.maxTor=dc.jntCtrlLoco.maxTor;
			jntCtrl.en=dc.jntCtrlLoco.enable;
			// ==关节约束
			For(dc.drvNums){
				Alg::clip(jntCtrl.j[i], jntLims.minPos[i], jntLims.maxPos[i]);
				Alg::clip(jntCtrl.w[i], jntLims.maxVel[i]);
				Alg::clip(jntCtrl.kp[i], 0, jntLims.kpMax[i]);
				Alg::clip(jntCtrl.kd[i], 0, jntLims.kdMax[i]);

				float tmp=jntCtrl.kp[i]*(jntCtrl.j[i]-dc.jntSensLoco.j[i])
						 +jntCtrl.kd[i]*(jntCtrl.w[i]-dc.jntSensLoco.w[i]);
				motCtrl.tor[i]=jntCtrl.t[i] +jntFil[i].filt(tmp);//准备下面jnt转mot
				// motCtrl.tor[i]=jntCtrl.t[i] +tmp;//准备下面jnt转mot
				Alg::clip(motCtrl.tor[i], jntCtrl.maxTor[i]);//用户约束
				Alg::clip(motCtrl.tor[i], jntLims.maxTor[i]);//接口约束
			}

			// ==关节转电机
			motCtrl.pos=jntCtrl.j;
			motCtrl.vel=jntCtrl.w;
		#ifndef DefSim
			JntMap::dwdate(motCtrl.pos, motCtrl.vel, motCtrl.tor);
		#endif
			For(dc.drvNums){
				motTgtPtrMaped[i]->pos=motCtrl.pos[i];
				motTgtPtrMaped[i]->vel=motCtrl.vel[i];
				motTgtPtrMaped[i]->tor=motCtrl.tor[i];
				motTgtPtrMaped[i]->enabled=jntCtrl.en[i];
			}
			For(dc.fingerDofs[0]){
				fingerTgts[i].pos=jntCtrl.fingerJ[0][i];
			}
			For(dc.fingerDofs[1]){
				fingerTgts[i+dc.fingerDofs[0]].pos=jntCtrl.fingerJ[1][i];
			}
		}else{//waitOp
			jntCtrl.j=dc.jntSensLoco.j;
			jntCtrl.w.setZero();
			jntCtrl.t.setZero();
			motCtrl.vel.setZero();
			motCtrl.tor.setZero();
			For(dc.drvNums){
				motCtrl.pos[i]=motActPtrMaped[i]->pos;
				motTgts[i].pos=motActs[i].pos;
				motTgts[i].vel=0;
				motTgts[i].tor=0;
				motTgts[i].enabled=0;
				jntFil[i].setBase(0);
			}
		}
		// 清错
		if(dc.cmd.key==230){
			For(dc.drvNums){
				if(motTgts[i].enabled==0){
					motTgts[i].enabled=-1;
				}
			}
		}
		if(cnt%2000==4){//不集中打印，以防影响实时性
			int id=0;
			cout<<">>下发 mot pos [l arm |r arm |neck+lumbar |l leg |r leg]:\n";
			For(limbNum){
				// for(int g=0;g<limbGroup[i];g++){
				// 	cout<<motTgtPtrMaped[id+g]->pos<<", ";
				// }
				for(int g=0;g<limbGroup[i];g++){
					cout<<motTgts[dc.idMap[id+g]].pos<<", ";
				}
				id+=limbGroup[i];
				if(limbGroup[i]){cout<<endl;}
			}
		}
		sdk.setMotorTarget(motTgts);
		sdk.setDigitTarget(fingerTgts);
	}
	void driverTaskClass::impClass::log(){
		if(cnt%logCnt==0){
			logSS.clear();logSS.str("");
			
			logSS<<"key\t"<<dc.cmd.key;
			logSS<<"\tmotStatus\t";
			For(dc.drvNums){logSS<<motActPtrMaped[i]->statusWord<<"\t";}
			logSS<<"actPos\t";
			For(dc.drvNums){logSS<<motActPtrMaped[i]->pos<<"\t";}
			logSS<<"actVel\t";
			For(dc.drvNums){logSS<<motActPtrMaped[i]->vel<<"\t";}
			logSS<<"actTor\t";
			For(dc.drvNums){logSS<<motActPtrMaped[i]->tor<<"\t";}
			logSS<<"actErr\t";
			For(dc.drvNums){logSS<<motActPtrMaped[i]->errorCode<<"\t";}
			logSS<<"actTemp\t";
			For(dc.drvNums){logSS<<motActPtrMaped[i]->temp<<"\t";}

			logSS<<"tgtPos\t";
			For(dc.drvNums){logSS<<motTgtPtrMaped[i]->pos<<"\t";}
			logSS<<"offVel\t";
			For(dc.drvNums){logSS<<motTgtPtrMaped[i]->vel<<"\t";}
			logSS<<"offTor\t";
			For(dc.drvNums){logSS<<motTgtPtrMaped[i]->tor<<"\t";}

			logSS<<"rpy\t";
			For(3){logSS<<sdkImu.rpy[i]<<"\t";}
			logSS<<"gyr\t";
			For(3){logSS<<sdkImu.gyr[i]<<"\t";}
			logSS<<"acc\t";
			For(3){logSS<<sdkImu.acc[i]<<"\t";}
			logger.log(logSS.str());
		}
	}
	void driverTaskClass::impClass::enforceDisable(){
		For(dc.drvNums){
			motTgts[i].pos=motActs[i].pos;
			motTgts[i].vel=0;
			motTgts[i].tor=0;
			motTgts[i].enabled=0;
		}
		sdk.setMotorTarget(motTgts);
	}
// =================================
	driverTaskClass::driverTaskClass():imp(*new impClass()){}
	driverTaskClass::~driverTaskClass(){
		delete &imp;
	}
	void driverTaskClass::init(const string &name,float dt){
		taskName=name;
		this->dt=dt;
		imp.init(dt);
	}
	bool driverTaskClass::step(){
		imp.update();
		imp.work();
		imp.dwdate();
		imp.log();
		return running;
	}
	void driverTaskClass::disable(){
		imp.enforceDisable();
	}
}//namespace

