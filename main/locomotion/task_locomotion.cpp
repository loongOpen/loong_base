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
#include"task_locomotion.h"
#include"nabo_locomotion/nabo.h"

namespace Task{
class locoTaskClass::impClass{
public:
	impClass();
	~impClass();
	void init(float dt);
	void update();
	void work();
	void dwdate();
	float dt;
	int cnt;
	bool ocuLost;
	
	Data::dataCenterClass &dc=Data::dataCenterClass::instance();

	Nabo::inputStruct in;
	Nabo::outputStruct out;
};
	locoTaskClass::impClass::impClass(){
		in.sens.init(dc.drvNums, dc.fingerDofs[0], dc.fingerDofs[1]);
		in.maniSdk.init(dc.armDof, dc.fingerDofs[0], dc.fingerDofs[1], dc.neckDof, dc.lumbarDof);
		in.jntSdk.init(dc.drvNums, dc.fingerDofs[0], dc.fingerDofs[1]);
		out.ctrl.init(dc.drvNums, dc.fingerDofs[0], dc.fingerDofs[1]);
		out.param.init(dc.drvNums);
	}
	locoTaskClass::impClass::~impClass(){
		dc.locoOpenFlag=0;
		dc.shmLocoOpenFlag.set(&dc.locoOpenFlag);
	}
	void locoTaskClass::impClass::init(float dt){
		this->dt=dt;
		dc.locoOpenFlag=1;
		dc.shmLocoOpenFlag.set(&dc.locoOpenFlag);
		Nabo::init(dt);
	}
	void locoTaskClass::impClass::update(){
		/*======================
				！！！！
		尽管共享内存与nabo的很多结构相同，但最好不要直接memcpy
				！！！！
		=======================*/
		if(dc.shmCmd.get(&dc.cmd)>-1000){
			in.cmd.key=dc.cmd.key;
			in.cmd.vx=dc.cmd.vx;
			in.cmd.vy=dc.cmd.vy;
			in.cmd.wz=dc.cmd.wz;
			in.cmd.zOff=dc.cmd.zOff;
			in.cmd.naviTap=dc.cmd.naviTap;
			ocuLost=0;
		}else if(ocuLost==0){
			ocuLost=1;
			in.cmd.key=-1;
			in.cmd.vx=0;
			in.cmd.vy=0;
			in.cmd.wz=0;
			print("ocu lost !");
		}

		dc.shmImu.get(&dc.imu);//不建议使用memcpy
		For3{
			in.sens.rpy[i]=dc.imu.rpy[i];
			in.sens.gyr[i]=dc.imu.gyr[i];
			in.sens.acc[i]=dc.imu.acc[i];
		}
		dc.jntSensLoco.getFromShm();//不建议使用memcpy
		in.sens.j=dc.jntSensLoco.j;
		in.sens.w=dc.jntSensLoco.w;
		in.sens.t=dc.jntSensLoco.t;
		in.sens.state=dc.jntSensLoco.state;
		dc.fingerSens.getFromShm();//不建议使用memcpy
		
		in.sens.finger[0]=dc.fingerSens.j[0];
		in.sens.finger[1]=dc.fingerSens.j[1];

		if(dc.locoJntSdk.getFromShm()>-200){//不建议使用memcpy
			in.jntSdk.state=dc.locoJntSdk.state;
			in.jntSdk.torLimitRate=dc.locoJntSdk.torLimitRate;
			in.jntSdk.filtRate=dc.locoJntSdk.filtRate;
			in.jntSdk.j=dc.locoJntSdk.j;
			in.jntSdk.w=dc.locoJntSdk.w;
			in.jntSdk.t=dc.locoJntSdk.t;
			in.jntSdk.kp=dc.locoJntSdk.kp;
			in.jntSdk.kd=dc.locoJntSdk.kd;
			in.jntSdk.finger[0]=dc.locoJntSdk.finger[0];
			in.jntSdk.finger[1]=dc.locoJntSdk.finger[1];
		}else{
			in.jntSdk.state=0;
		}

		if(dc.shmAppIn.get(&dc.appIn)>-5){//不建议使用memcpy
			in.app.dataLen=dc.appIn.dataLen;
			memcpy(in.app.data, dc.appIn.data, in.app.dataLen);
		}
	}
	void locoTaskClass::impClass::work(){
		cnt++;
		static int key=0;
		if(key!=in.cmd.key){
			print(in.cmd.key, in.cmd.vx, in.cmd.vy, in.cmd.wz, in.cmd.zOff);
			key=in.cmd.key;
		}
		if(ocuLost && cnt%5000==1){
			print("ocu loss !");
		}
		Nabo::step(in,out);
	}
	void locoTaskClass::impClass::dwdate(){
		if(out.ctrl.j.size()!=dc.drvNums ||
				out.ctrl.w.size()!=dc.drvNums ||
				out.ctrl.t.size()!=dc.drvNums ||
				out.param.kp.size()!=dc.drvNums ||
				out.param.kd.size()!=dc.drvNums ||
				out.param.maxTor.size()!=dc.drvNums)
		{
			stringstream ss;
			ss<<"task loco:控制层out内向量维度不匹配drvNum！";
			ss<<"task drvNum="<<dc.drvNums;
			ss<<",out size="<<out.ctrl.j.size();
			throw runtime_error(ss.str());
		}
		//不建议使用memcpy
		dc.jntCtrlLoco.j=out.ctrl.j;
		dc.jntCtrlLoco.w=out.ctrl.w;
		dc.jntCtrlLoco.t=out.ctrl.t;
		dc.jntCtrlLoco.kp=out.param.kp;
		dc.jntCtrlLoco.kd=out.param.kd;
		dc.jntCtrlLoco.maxTor=out.param.maxTor;

		if(out.ctrl.enFlag){
			For(dc.drvNums){
				dc.jntCtrlLoco.enable[i]=1;
			}
		}else{
			For(dc.drvNums){
				dc.jntCtrlLoco.enable[i]=0;
			}
		}
		dc.jntCtrlLoco.setToShm();
		//外传信息
		dc.locoInfo.planKey=out.info.planKey;
		memcpy(&dc.locoInfo.planName, &out.info.planName, 16);
		dc.locoInfo.state[0]=out.info.state[0];
		dc.locoInfo.state[1]=out.info.state[1];

		dc.locoInfo.bodyP=out.info.bodyP;
		dc.locoInfo.bodyV2F=out.info.bodyV2F;

		dc.locoInfo.tipPRpy2F[0]=out.info.tipPRpy2F[0];
		dc.locoInfo.tipPRpy2F[1]=out.info.tipPRpy2F[1];
		dc.locoInfo.tipVW2F[0]=out.info.tipVW2F[0];
		dc.locoInfo.tipVW2F[1]=out.info.tipVW2F[1];
		dc.locoInfo.tipFM2F[0]=out.info.tipFM2F[0];
		dc.locoInfo.tipFM2F[1]=out.info.tipFM2F[1];
		
		dc.shmLocoInfo.set(&dc.locoInfo);
	}
// =================================
	locoTaskClass::locoTaskClass():imp(*new impClass()){

	}
	locoTaskClass::~locoTaskClass(){
		delete &imp;
	}
	void locoTaskClass::init(const string &name,float dt){
		taskName=name;
		this->dt=dt;
		imp.init(dt);
	}
	bool locoTaskClass::step(){
		imp.update();
		imp.work();
		imp.dwdate();
		return running;
	}

}//namespace

