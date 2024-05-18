#include <string>
#include <cstring>
#include <iostream>
#include <sstream>
#include <aris.hpp>
#include <aris/ext/json.hpp>

int main(int argc, char *argv[]){
	
	aris::core::Socket server;

	

	server.setOnReceivedMsg([](aris::core::Socket* socket, aris::core::Msg& msg)->int {
		// 得到标定所需的数据 //
		std::stringstream msg_stm(msg.toString());
		std::vector<double> data;
		std::string num;
		while (msg_stm >> num) {
			try {
				data.push_back(std::stod(num));
			}
			catch (...) {
			}
		}
		const int n = data.size() / 12;
		if (data.size() % 12 || n < 8) {
			ARIS_COUT << "[ERROR] : input data num not correct:" << data.size() << std::endl;
			socket->sendMsg(aris::core::Msg("[ERROR] : input data num not correct : " + std::to_string(data.size())));
			return 0;
		}

		std::vector<double> rbt_pes, vis_pes;

		rbt_pes.assign(data.begin(), data.begin() + data.size() / 2);
		vis_pes.assign(data.begin() + data.size() / 2, data.end());

		ARIS_COUT << "recv robot data: " << std::endl;
		aris::dynamic::dsp(n, 6, rbt_pes.data());
		ARIS_COUT << "recv vision data: " << std::endl;
		aris::dynamic::dsp(n, 6, vis_pes.data());
		// 得到标定配置
		std::ifstream f(aris::core::logExeDirectory() / "eye_in_server_config.json");
		auto config = nlohmann::json::parse(f);
		f.close();

		// 根据单位修正输入数据
		std::vector<double> rbt_pqs(7*n), vis_pqs(7*n);

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
			rbt_pos_ratio = get_pos_ratio(config["robot_target_position_unit"].get<std::string>());
			rbt_angle_ratio = get_angle_ratio(config["robot_target_angle_unit"].get<std::string>());
			vis_pos_ratio = get_pos_ratio(config["vision_position_unit"].get<std::string>());
			vis_angle_ratio = get_angle_ratio(config["vision_angle_unit"].get<std::string>());
			calib_pos_ratio = get_pos_ratio(config["calib_position_unit"].get<std::string>());
			calib_angle_ratio = get_angle_ratio(config["calib_angle_unit"].get<std::string>());
		}
		catch (...) {
			ARIS_COUT << "[ERROR] : invalid config unit"  << std::endl;
			socket->sendMsg(aris::core::Msg("[ERROR] : invalid config unit "));
			return 0;
		}
		
		for (int i = 0; i < n; ++i) {
			for (int j = 0; j < 3; ++j) {
				rbt_pes[aris::dynamic::at(i, j, 6)] *= rbt_pos_ratio;
				vis_pes[aris::dynamic::at(i, j, 6)] *= vis_pos_ratio;
				rbt_pes[aris::dynamic::at(i, j + 3, 6)] *= rbt_angle_ratio;
				vis_pes[aris::dynamic::at(i, j + 3, 6)] *= vis_angle_ratio;
			}

			aris::dynamic::s_pe2pq(rbt_pes.data() + 6 * i, rbt_pqs.data() + 7 * i, "321");
			aris::dynamic::s_pe2pq(vis_pes.data() + 6 * i, vis_pqs.data() + 7 * i, "321");
		}


		// 标定结果
		double result_pq[7];
		std::vector<double> mem(n * n * 16);
		aris::dynamic::s_eye_in_hand_calib(n, vis_pqs.data(), rbt_pqs.data(), result_pq, mem.data());
		if (result_pq[6] < 0)
			aris::dynamic::s_nv(4, -1.0, result_pq + 3);

		double output_pq[7]{ result_pq[0],result_pq[1] ,result_pq[2] ,result_pq[6] ,result_pq[3],result_pq[4] ,result_pq[5] };

		ARIS_COUT << "Calib result:" << std::endl;
		aris::dynamic::dsp(1, 7, output_pq);



		// change mechmind //
		f.open(config["mechmind_path"].get<std::string>());
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
		if (pos_diff > config["calib_position_max_diff_meter"].get<double>()
			|| angle_diff > config["calib_angle_max_diff_rad"].get<double>()) {
		
			ARIS_COUT << "[ERROR] : too large difference" << std::endl;
			socket->sendMsg(aris::core::Msg("[ERROR] : too large difference"));
			return 0;
		}

		js["depthInBase"] = std::vector<double>(output_pq, output_pq + 7);
		
		std::ofstream f2;
		f2.open(config["mechmind_path"].get<std::string>());
		f2.clear();
		f2 << js.dump(2);
		f2.close();

		socket->sendMsg(aris::core::Msg("success"));
		return 0;
	});

	server.setOnReceivedConnection([](aris::core::Socket* socket, const char* ip, int port)->int {
		ARIS_COUT  << " receive connection from:" << ip << ":" << port << std::endl;
		return 0;
	});

	server.setOnLoseConnection([](aris::core::Socket* socket)->int {
		ARIS_COUT << " lose connection" << std::endl;
		for (;;)
		{
			try
			{
				socket->startServer(socket->port());
				break;
			}
			catch (std::runtime_error& e)
			{
				ARIS_COUT << e.what() << " will try to restart server socket in 1s" << std::endl;
				std::this_thread::sleep_for(std::chrono::seconds(1));
			}
		}
		ARIS_COUT << " restart successful" << std::endl;
		return 0;
	});

	std::ifstream f(aris::core::logExeDirectory() / "eye_in_server_config.json");
	auto config = nlohmann::json::parse(f);
	server.startServer(config["server_port"].get<std::string>());



	for (;;) {
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}



	return 0;
}