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
成员状态计数互相独立，但所需的交互的数据指向同一片内存。
=====================================================*/

#include"data_center.h"
#include"iopack.h"

using namespace std;

namespace Data{
	int shmBaseDataStruct::shmCreat(int&mark){
		if(datas.size()!=sizes.size()){
			stringstream ss;
			ss<<"data center内datas与sizes不匹配，添加shm目标mark="<<mark;
			throw runtime_error(ss.str());
			exit(-1);
		}
		dataSizePrivate=0;
		for(auto&it:sizes){
			dataSizePrivate+=it;
		}
		shm.creat(mark,dataSizePrivate);
		return dataSizePrivate;
	}
// =============================
	int jntSensStruct::init(int &mark, int jntNum){
		j.setZero(jntNum);
		w.setZero(jntNum);
		t.setZero(jntNum);
		state.setZero(jntNum);
		errCode.setZero(jntNum);
		temperature.setZero(jntNum);

		datas.emplace_back(j.data());
		datas.emplace_back(w.data());
		datas.emplace_back(t.data());
		datas.emplace_back(state.data());
		datas.emplace_back(errCode.data());
		datas.emplace_back(temperature.data());

		sizes.emplace_back(sizeof(j.value())*jntNum);
		sizes.emplace_back(sizeof(w.value())*jntNum);
		sizes.emplace_back(sizeof(t.value())*jntNum);
		sizes.emplace_back(sizeof(state.value())*jntNum);
		sizes.emplace_back(sizeof(errCode.value())*jntNum);
		sizes.emplace_back(sizeof(temperature.value())*jntNum);
		return shmCreat(mark);
	}
	int jntCtrlStruct::init(int &mark, int jntNum, int fingerDofLeft, int fingerDofRight){
		j.setZero(jntNum);
		w.setZero(jntNum);
		t.setZero(jntNum);
		kp.setZero(jntNum);
		kd.setZero(jntNum);
		maxTor.setZero(jntNum);
		enable.setZero(jntNum);
		fingerJ[0].setZero(fingerDofLeft);
		fingerJ[1].setZero(fingerDofRight);

		datas.emplace_back(j.data());
		datas.emplace_back(w.data());
		datas.emplace_back(t.data());
		datas.emplace_back(kp.data());
		datas.emplace_back(kd.data());
		datas.emplace_back(maxTor.data());
		datas.emplace_back(enable.data());
		datas.emplace_back(fingerJ[0].data());
		datas.emplace_back(fingerJ[1].data());
		datas.emplace_back(&inCharge);

		sizes.emplace_back(sizeof(j.value())*jntNum);
		sizes.emplace_back(sizeof(w.value())*jntNum);
		sizes.emplace_back(sizeof(t.value())*jntNum);
		sizes.emplace_back(sizeof(kp.value())*jntNum);
		sizes.emplace_back(sizeof(kd.value())*jntNum);
		sizes.emplace_back(sizeof(maxTor.value())*jntNum);
		sizes.emplace_back(sizeof(enable.value())*jntNum);
		sizes.emplace_back(sizeof(fingerJ[0].value())*fingerDofLeft);
		sizes.emplace_back(sizeof(fingerJ[1].value())*fingerDofRight);
		sizes.emplace_back(sizeof(inCharge));
		return shmCreat(mark);
	}

	int locoJntSdkStruct::init(int &mark, int jntNum, int fingerDofLeft, int fingerDofRight){
		j.setZero(jntNum);
		w.setZero(jntNum);
		t.setZero(jntNum);
		kp.setZero(jntNum);
		kd.setZero(jntNum);
		finger[0].setZero(fingerDofLeft);
		finger[1].setZero(fingerDofRight);

		datas.emplace_back(&checker);
		datas.emplace_back(&size);
		datas.emplace_back(&state);
		datas.emplace_back(&torLimitRate);
		datas.emplace_back(&filtRate);
		datas.emplace_back(j.data());
		datas.emplace_back(w.data());
		datas.emplace_back(t.data());
		datas.emplace_back(kp.data());
		datas.emplace_back(kd.data());
		datas.emplace_back(finger[0].data());
		datas.emplace_back(finger[1].data());

		sizes.emplace_back(sizeof(checker));
		sizes.emplace_back(sizeof(size));
		sizes.emplace_back(sizeof(state));
		sizes.emplace_back(sizeof(torLimitRate));
		sizes.emplace_back(sizeof(filtRate));
		sizes.emplace_back(sizeof(j.value())*jntNum);
		sizes.emplace_back(sizeof(w.value())*jntNum);
		sizes.emplace_back(sizeof(t.value())*jntNum);
		sizes.emplace_back(sizeof(kp.value())*jntNum);
		sizes.emplace_back(sizeof(kd.value())*jntNum);
		sizes.emplace_back(sizeof(finger[0].value())*fingerDofLeft);
		sizes.emplace_back(sizeof(finger[1].value())*fingerDofRight);
		size=0;
		for(auto&it:sizes){
			size+=it;
		}
		
		return shmCreat(mark);
	};
	int fingerSensStruct::init(int &mark, int fingerDofLeft, int fingerDofRight){//双手区分
		j[0].setZero(fingerDofLeft);
		j[1].setZero(fingerDofRight);
		w[0].setZero(fingerDofLeft);
		w[1].setZero(fingerDofRight);

		datas.emplace_back(j[0].data());
		datas.emplace_back(j[1].data());
		datas.emplace_back(w[0].data());
		datas.emplace_back(w[1].data());

		sizes.emplace_back(sizeof(j[0].value())*fingerDofLeft);
		sizes.emplace_back(sizeof(j[1].value())*fingerDofRight);
		sizes.emplace_back(sizeof(w[0].value())*fingerDofLeft);
		sizes.emplace_back(sizeof(w[1].value())*fingerDofRight);
		return shmCreat(mark);
	}
// =================================================
	dataCenterClass::dataCenterClass(){
		Ini::iniClass iniDrv("../config/driver.ini");
		drvNums=iniDrv["drvNums"];
		armDof=iniDrv["armDof"];
		iniDrv.getArray("fingerDofs",fingerDofs);
		fingerDofBoth=fingerDofs[0]+fingerDofs[1];
		neckDof=iniDrv["neckDof"];
		lumbarDof=iniDrv["lumbarDof"];
		legDof=iniDrv["legDof"];
		maniDrvNums=armDof*2 +neckDof +lumbarDof;

		idMap.resize(drvNums);
		iniDrv.getArray("idMap", idMap.data(), drvNums);

		int mark=2178;//随机初值，减小和其他程序冲突
		jntSensLoco.init(mark, drvNums);
		jntCtrlLoco.init(mark, drvNums, fingerDofs[0], fingerDofs[1]);
		locoJntSdk.init(mark, drvNums, fingerDofs[0], fingerDofs[1]);
		fingerSens.init(mark, fingerDofs[0], fingerDofs[1]);

		shmImu.creat(mark, sizeof(imuStruct));
		shmCmd.creat(mark, sizeof(cmdStruct));
		shmAppIn.creat(mark, sizeof(appStruct));
		shmAppOut.creat(mark, sizeof(appStruct));
		shmLocoInfo.creat(mark, sizeof(locoInfoStruct));

		shmLocoOpenFlag.creat(mark, sizeof(int));
	}
	dataCenterClass& dataCenterClass::instance(){
		static dataCenterClass singtn;
		return singtn;
	}
}//namespace

