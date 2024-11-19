#include <iostream>
#include <regex>
#include <charconv>

#include <aris.hpp>

auto test_f()->void {
	auto& cs = aris::server::ControlServer::instance();
	aris::core::fromXmlFile(cs, "C:\\Users\\py033\\Desktop\\test_aris_model_inv\\puma.xml");
	//aris::core::fromXmlFile(cs, "..\\ur.xml");
	//aris::core::fromXmlFile(cs, "..\\scara.xml");
	cs.init();
	cs.start();

	auto mm = dynamic_cast<aris::dynamic::MultiModel*>(&cs.model());
	auto model = dynamic_cast<aris::dynamic::Model*>(&mm->subModels()[0]);

	double output[6] = { 495.808,35.415,711.408,183.462,20.291,180.861 };
	double input[6]{ 0 };


	//-----------------------------------第三组解反向计算不正确---------------------------------------//
	aris::dynamic::s_nv(3, 0.001, output);
	aris::dynamic::s_nv(3, aris::PI / 180.0, output + 3);
	std::cout << "------------------------output-------------------" << std::endl;
	aris::dynamic::dsp(1, 6, output);
	for (size_t i = 0; i < 8; i++)
	{
		if (model->inverseKinematics(output, input, i))
			std::cout << "inv faile in solver" << std::to_string(i) << std::endl;
		else
		{
			std::cout << "solver " << std::to_string(i);
			aris::dynamic::dsp(1, 6, input);
		}

	}

	double input0[6]{ 0.075112, -0.036990,   0.168264, -0.022744,   1.085726 ,  0.024386 };
	double input1[6]{ 0.075112, -0.036990,   0.168264,   3.118849 ,-1.085726, -3.117206 };
	double input2[6]{ 0.075112,   1.586148, -3.075843, -0.047713,   2.706153, -0.029487 };
	double input3[6]{ 0.075112,   1.586148, -3.075843,   3.093879, -2.706153,   3.112106 };
	double input4[6]{ -3.066480, -1.567315, -0.084067,   3.093826,   2.706668, -0.029545 };
	double input5[6]{ -3.066480, -1.567315, -0.084067, -0.047766, -2.706668,  3.112048 };
	double input6[6]{ -3.066480, -0.196751, -2.823511,   3.120916,   1.338272,   0.018545 };
	double input7[6]{ -3.066480, -0.196751, -2.823511, -0.020676, -1.338272, -3.123047 };

	//-----------------------------------按照反解算的值来求根    计算有误---------------------------------------//
	std::cout << "/----------------------------------------/" << std::endl;
	std::cout << "input0 which_root:" << model->whichInverseRoot(output, input0) << std::endl;
	std::cout << "input1 which_root:" << model->whichInverseRoot(output, input1) << std::endl;
	std::cout << "input2 which_root:" << model->whichInverseRoot(output, input2) << std::endl;
	std::cout << "input3 which_root:" << model->whichInverseRoot(output, input3) << std::endl;
	std::cout << "input4 which_root:" << model->whichInverseRoot(output, input4) << std::endl;
	std::cout << "input5 which_root:" << model->whichInverseRoot(output, input5) << std::endl;
	std::cout << "input6 which_root:" << model->whichInverseRoot(output, input6) << std::endl;
	std::cout << "input7 which_root:" << model->whichInverseRoot(output, input7) << std::endl;


	cs.runCmdLine();
}


int main(int argc, char *argv[]){
	test_f();



	aris::core::setLanguage(1);

	auto& cs = aris::server::ControlServer::instance();

	cs.resetMaster(aris::control::createDefaultEthercatMaster(6, 0, 0).release());

	cs.resetController(
		aris::control::createDefaultEthercatController(6, 0, 0, dynamic_cast<aris::control::EthercatMaster&>(cs.master())
		).release()
	);
	for (int i = 0; i < 6; ++i) {
		cs.controller().motorPool()[i].setMaxPos(3.14);
		cs.controller().motorPool()[i].setMinPos(-3.14);
		cs.controller().motorPool()[i].setMaxVel(3.14);
		cs.controller().motorPool()[i].setMinVel(-3.14);
		cs.controller().motorPool()[i].setMaxAcc(100);
		cs.controller().motorPool()[i].setMinAcc(-100);
	}


	aris::dynamic::PumaParam puma_param;
	puma_param.d1 = 0.3;
	puma_param.a1 = 0.1;
	puma_param.a2 = 0.4;
	puma_param.a3 = 0.05;
	puma_param.d3 = 0.0;
	puma_param.d4 = 0.35;
	puma_param.install_method = 1;

	puma_param.base2ref_pe[0] = 1;
	puma_param.base2ref_pe[1] = 2;
	puma_param.base2ref_pe[2] = 3;
	// 安装方式
	// 0, 正常安装，零位时末端法兰盘朝向：地面 x 轴，零位时末端1轴朝向：地面 z 轴
	// 1，顶部吊装，零位时末端法兰盘朝向：地面 x 轴，零位时末端1轴朝向：地面-z 轴
	// 2，侧装向上，零位时末端法兰盘朝向：地面 z 轴，零位时末端1轴朝向：地面 x 轴
	// 3，侧装向下，零位时末端法兰盘朝向：地面-z 轴，零位时末端1轴朝向：地面 x 轴
	auto puma = aris::dynamic::createModelPuma(puma_param);

	puma->forwardKinematics();
	double pe[6];
	puma->getInputPos(pe);
	aris::dynamic::dsp(1, 6, pe);

	cs.resetModel(puma.release());

	cs.resetPlanRoot(aris::plan::createDefaultPlanRoot().release());
	std::cout << aris::core::toXmlString(cs) << std::endl;

	try
	{
		cs.interfacePool().add<aris::server::HttpInterface>("http", "8001", "C:/Users/py033/WebstormProjects/RobotControllerHMI/client/build");
		cs.interfacePool().add<aris::server::ProgramWebInterface>();
		
		cs.init();
		cs.open();
		cs.start();

		// 读取数据 //
		//double data[6];
		//cs.controller().ftSensorPool()[0].getFtData(data);

		//std::cout << aris::core::toXmlString(cs) << std::endl;

		cs.runCmdLine();
		//aris::core::toXmlFile(cs, "C:\\Users\\py033\\Desktop\\test.xml");
		//aris::core::fromXmlFile(cs, "C:\\Users\\py033\\Desktop\\test.xml");
	}
	catch (std::exception &e)
	{
		std::cout << e.what() << std::endl;
	}
	

	

	return 0;
}