//#######################################################
//################      h文件       ######################
//#######################################################

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
// #pragma once
// #include"task_base.h"
// namespace Task{
// class exampleTaskClass:public periodicTaskClass{
// public:
// 	exampleTaskClass();
// 	~exampleTaskClass();

// 	void init(float dt)override final;
// 	bool step()override final;
// private:
// 	exampleTaskClass(const exampleTaskClass&)=delete;
// 	exampleTaskClass& operator=(const exampleTaskClass&)=delete;
// 	class impClass;
// 	impClass&imp;
// };
// }//namespace



//#######################################################
//################      cpp文件       ####################
//#######################################################

/*=========== ***doc description @ yyp*** ===========
This is part of Nabo (Naughty Boy, 小炮), an open project for the control of biped robot，
Copyright (C) 2024 YYP, Shanghai Jiao Tong University, under MIT License.
Feel free to use in any purpose, and cite Nabo or 小炮 in any style, to contribut to the advancement of the community.
<https://github.com/tryingfly/nabo>
<https://www.zhihu.com/column/c_1746480685987373057>

=====================================================*/

// #include"task_example.h"

// namespace Task{
// class exampleTaskClass::impClass{
// public:
// 	impClass();
//	void init(float dt);
// 	void update();
// 	void work();
// 	void dwdate();
// 	float dt;
// 	Data::dataCenterClass dc;
// };
	// exampleTaskClass::impClass::impClass(){
		
	// }
// 	void exampleTaskClass::impClass::init(flaot dt){
//		this->dt=dt;
// 	}
// 	void exampleTaskClass::impClass::update(){

// 	}
// 	void exampleTaskClass::impClass::work(){

// 	}
// 	void exampleTaskClass::impClass::dwdate(){

// 	}
// // =================================
// 	exampleTaskClass::exampleTaskClass():imp(*new impClass()){

// 	}
// 	exampleTaskClass::~exampleTaskClass(){
// 		delete &imp;
// 	}
// 	void exampleTaskClass::init(float dt){
// 		this->dt=dt;
// 		imp.init(dt);
// 	}
// 	bool exampleTaskClass::step(){
// 		imp.update();
// 		imp.work();
// 		imp.dwdate();
// 		return running;
// 	}

// }//namespace

