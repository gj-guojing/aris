#include <string>
#include <cstring>
#include <iostream>
#include <sstream>
#include <aris.hpp>
#include <aris/ext/json.hpp>

class CalibEyeInHandPlan : public aris::core::CloneObject<CalibEyeInHandPlan, aris::plan::Plan> {
public:

	auto virtual prepareNrt()->void override {
		option() |= aris::plan::Plan::Option::NOT_RUN_EXECUTE_FUNCTION;
		option() |= aris::plan::Plan::Option::NOT_RUN_COLLECT_FUNCTION;
		
		// 获取数据 //
		auto rbt_pes = this->matrixParam("rbt_pes");
		auto vis_pes = this->matrixParam("vis_pes");
		aris::dynamic::dsp(rbt_pes.m(), rbt_pes.n(), rbt_pes.data());
		aris::dynamic::dsp(vis_pes.m(), vis_pes.n(), vis_pes.data());
		
		if (rbt_pes.m() != vis_pes.m() || rbt_pes.n() != 6 || vis_pes.n() != 6) {
			this->setPrepareRetCode(-1);
			this->setPrepareRetMsg("vis_pes or rbt_pes size not correct");
			return;
		}

		ARIS_COUT_PLAN(this) << "recv robot data: " << std::endl;
		aris::dynamic::dsp(rbt_pes.m(), rbt_pes.n(), rbt_pes.data());
		ARIS_COUT_PLAN(this) << "recv vision data: " << std::endl;
		aris::dynamic::dsp(vis_pes.m(), vis_pes.n(), vis_pes.data());
		
		// 修正单位 //
		auto get_pos_ratio = [](std::string unit)->double {
			if (unit == "m") {
				return 1.0;
			}
			else if (unit == "mm") {
				return 0.001;
			}
			throw std::runtime_error("invalid unit");
		};
		auto get_angle_ratio = [](std::string unit)->double {
			if (unit == "deg") {
				return aris::PI / 180;
			}
			else if (unit == "rad") {
				return 1.0;
			}
			else if (unit == "quad") {
				return 1.0;
			}
			throw std::runtime_error("invalid unit");
		};

		double rbt_pos_ratio, rbt_angle_ratio, vis_pos_ratio, vis_angle_ratio, calib_pos_ratio, calib_angle_ratio;
		try {
			rbt_pos_ratio = get_pos_ratio(stringParam("rbt_pos_unit"));
			rbt_angle_ratio = get_angle_ratio(stringParam("rbt_angle_unit"));
			vis_pos_ratio = get_pos_ratio(stringParam("vis_pos_unit"));
			vis_angle_ratio = get_angle_ratio(stringParam("vis_angle_unit"));
			calib_pos_ratio = get_pos_ratio(stringParam("clb_pos_unit"));
			calib_angle_ratio = get_angle_ratio(stringParam("rbt_angle_unit"));
		}
		catch (...) {
			ARIS_COUT << "[ERROR] : invalid config unit" << std::endl;
			this->setPrepareRetCode(-2);
			this->setPrepareRetMsg("invalid unit");
			return;
		}

		// 转换成 pq //
		std::vector<double> rbt_pqs(7 * rbt_pes.m()), vis_pqs(7 * rbt_pes.m());
		for (int i = 0; i < rbt_pes.m(); ++i) {
			for (int j = 0; j < 3; ++j) {
				rbt_pes(i, j) *= rbt_pos_ratio;
				vis_pes(i, j) *= vis_pos_ratio;
				rbt_pes(i, j + 3) *= rbt_angle_ratio;
				vis_pes(i, j + 3) *= vis_angle_ratio;
			}

			aris::dynamic::s_pe2pq(rbt_pes.data() + 6 * i, rbt_pqs.data() + 7 * i, "321");
			aris::dynamic::s_pe2pq(vis_pes.data() + 6 * i, vis_pqs.data() + 7 * i, "321");
		}

		// 标定算法 //
		double result_pq[7];
		std::vector<double> mem(rbt_pes.m() * rbt_pes.m() * 16);
		aris::dynamic::s_eye_in_hand_calib(rbt_pes.m(), vis_pqs.data(), rbt_pqs.data(), result_pq, mem.data());
		if (result_pq[6] < 0)
			aris::dynamic::s_nv(4, -1.0, result_pq + 3);

		double output_pq[7]{ result_pq[0],result_pq[1] ,result_pq[2] ,result_pq[6] ,result_pq[3],result_pq[4] ,result_pq[5] };

		ARIS_COUT << "Calib result:" << std::endl;
		aris::dynamic::dsp(1, 7, output_pq);

		// change mechmind //
		auto mechmind_path = std::filesystem::path(stringParam("mech_path"));

		if (!mechmind_path.is_absolute()) {
			mechmind_path = aris::core::logExeDirectory() / mechmind_path;
		}

		std::cout << "mechmind path:" << mechmind_path << std::endl;

		std::ifstream f(mechmind_path);
		auto js = nlohmann::json::parse(f);
		f.close();

		std::vector<double> origin_calib = js["depthInBase"].get<std::vector<double>>();


		if (output_pq[3] * origin_calib[3] < 0)
			aris::dynamic::s_nv(4, -1.0, output_pq + 3);

		double pos_diff = std::sqrt(
			(origin_calib[0] - output_pq[0]) * (origin_calib[0] - output_pq[0]) +
			(origin_calib[1] - output_pq[1]) * (origin_calib[1] - output_pq[1]) +
			(origin_calib[2] - output_pq[2]) * (origin_calib[2] - output_pq[2])
		);
		double angle_diff = std::sqrt(
			(origin_calib[3] - output_pq[3]) * (origin_calib[3] - output_pq[3]) +
			(origin_calib[4] - output_pq[4]) * (origin_calib[4] - output_pq[4]) +
			(origin_calib[5] - output_pq[5]) * (origin_calib[5] - output_pq[5]) +
			(origin_calib[6] - output_pq[6]) * (origin_calib[6] - output_pq[6])
		);

		ARIS_COUT << "diff : " << pos_diff << "  " << angle_diff << std::endl;
		if (pos_diff > doubleParam("max_clb_pos_err") || angle_diff > doubleParam("max_clb_angle_err")) {
			ARIS_COUT << "[ERROR] : too large difference" << std::endl;
			this->setPrepareRetCode(-3);
			this->setPrepareRetMsg("too large err");
			return;
		}

		js["depthInBase"] = std::vector<double>(output_pq, output_pq + 7);

		std::ofstream f2;
		f2.open(mechmind_path);
		f2.clear();
		f2 << js.dump(2);
		f2.close();
	}

	virtual ~CalibEyeInHandPlan() = default;
	explicit CalibEyeInHandPlan(const std::string& name = "calib_eye_in_hand") {
		aris::core::fromXmlString(command(),
			"<Command name=\"calib_eye_in_hand\">"
			"	<GroupParam>"
			"		<Param name=\"limit_time\" default=\"5000\"/>"
			"		<Param name=\"rbt_pes\" default=\"{0,0,0,0,0,0}\"/>"
			"		<Param name=\"vis_pes\" default=\"{0,0,0,0,0,0}\"/>"
			"		<Param name=\"rbt_pos_unit\" default=\"mm\"/>"
			"		<Param name=\"rbt_angle_unit\" default=\"deg\"/>"
			"		<Param name=\"vis_pos_unit\" default=\"mm\"/>"
			"		<Param name=\"vis_angle_unit\" default=\"rad\"/>"
			"		<Param name=\"clb_pos_unit\" default=\"m\"/>"
			"		<Param name=\"clb_angle_unit\" default=\"rad\"/>"
			"		<Param name=\"max_clb_pos_err\" default=\"0.01\"/>"
			"		<Param name=\"max_clb_angle_err\" default=\"0.05\"/>"
			"		<Param name=\"mech_path\" default=\"extri_param.json\"/>"
			"	</GroupParam>"
			"</Command>");
	}
};
class SysCmdPlan : public aris::core::CloneObject<SysCmdPlan, aris::plan::Plan> {
public:

	auto virtual prepareNrt()->void override {
		option() |= aris::plan::Plan::Option::NOT_RUN_EXECUTE_FUNCTION;
		option() |= aris::plan::Plan::Option::NOT_RUN_COLLECT_FUNCTION;

		// 获取数据 //
		system(stringParam("call").data());
	}

	virtual ~SysCmdPlan() = default;
	explicit SysCmdPlan(const std::string& name = "sys_cmd") {
		aris::core::fromXmlString(command(),
			"<Command name=\"sys_cmd\">"
			"	<GroupParam>"
			"		<Param name=\"call\" default=\"aaa\"/>"
			"	</GroupParam>"
			"</Command>");
	}
};

ARIS_REGISTRATION{
	aris::core::class_<CalibEyeInHandPlan>("CalibEyeInHandPlan")
			.inherit<aris::plan::Plan>()
			;
	
	aris::core::class_<SysCmdPlan>("SysCmdPlan")
			.inherit<aris::plan::Plan>()
			;
}

int main(int argc, char *argv[]){
	auto& cs = aris::server::ControlServer::instance();
	
	auto cs_xml_path = aris::core::logExeDirectory() / std::filesystem::path("cs.xml");


	try {
		aris::core::fromXmlFile(cs, cs_xml_path);
	}
	catch (...) {
		ARIS_COUT << "WARNING: FAILED TO FIND cs.xml File, use default param" << std::endl;
		cs.planRoot().planPool().add<CalibEyeInHandPlan>();
		cs.planRoot().planPool().add<SysCmdPlan>();
		cs.interfacePool().add<aris::server::WebInterface>("tcp_interface", "5870", aris::core::Socket::Type::TCP);
		aris::core::toXmlFile(cs, cs_xml_path);
	}

	cs.init();
	cs.start();
	cs.open();
	cs.runCmdLine();

	return 0;
}