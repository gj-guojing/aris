﻿#include <cstring>
#include <thread>
#include <algorithm>
#include <memory>
#include <cinttypes>
#include <queue>

#include <aris/core/core.hpp>
#include <aris/control/control.hpp>

#include "aris/ext/json.hpp"
#include "aris/ext/fifo_map.hpp"
#include "aris/server/control_server.hpp"
#include "aris/server/api.hpp"

namespace aris::server{
	auto TransferModelController::updateDataController2Model(
		const std::vector<std::uint64_t>& options, 
		const aris::control::Controller* controller, 
		aris::dynamic::ModelBase* model)->void
	{
		for (std::size_t i = 0; i < controller->motorPool().size(); ++i) {
			auto& cm = controller->motorPool()[i];
			if ((options[i] & aris::plan::Plan::UPDATE_MODEL_POS_FROM_CONTROLLER))
				model->setInputPosAt(cm.targetPos(), i);
			if ((options[i] & aris::plan::Plan::UPDATE_MODEL_VEL_FROM_CONTROLLER))
				model->setInputVelAt(cm.targetVel(), i);
		}
	}
	auto TransferModelController::updateDataModel2Controller(
		const std::vector<std::uint64_t>& options, 
		const aris::dynamic::ModelBase* model,
		aris::control::Controller* controller) ->void
	{
		for (std::size_t i = 0; i < controller->motorPool().size(); ++i) {
			auto& cm = controller->motorPool()[i];
			if ((options[i] & aris::plan::Plan::USE_TARGET_POS))
				cm.setTargetPos(model->inputPosAt(i));
			if ((options[i] & aris::plan::Plan::USE_TARGET_VEL))
				cm.setTargetVel(model->inputVelAt(i));
			if ((options[i] & aris::plan::Plan::USE_TARGET_TOQ))
				cm.setTargetToq(model->inputFceAt(i));
			if ((options[i] & aris::plan::Plan::USE_OFFSET_VEL))
				cm.setOffsetVel(model->inputVelAt(i));
			if ((options[i] & aris::plan::Plan::USE_OFFSET_TOQ))
				cm.setOffsetToq(model->inputFceAt(i));
		}
	}
	
	struct ControlServer::Imp{
		enum { CMD_POOL_SIZE = 10000 };
		
		struct InternalData{
			std::shared_ptr<aris::plan::Plan> plan_;
			std::function<void(aris::plan::Plan&)> post_callback_;
			bool has_prepared_{ false };

			~InternalData()	{
				// step 4a. 同步收集4a //
				if (has_prepared_ && (!(plan_->option() & aris::plan::Plan::NOT_RUN_COLLECT_FUNCTION))){
					ARIS_LOG(aris::core::LogLvl::kDebug, 0, { "server collect cmd %ji" }, plan_->cmdId());
					plan_->collectNrt();
				}

				// step 5a & 5b //
				if (post_callback_)post_callback_(*plan_);
			}
		};
		struct ChanelData {
			// 实时循环中的轨迹参数 //
			std::shared_ptr<InternalData> internal_data_queue_[CMD_POOL_SIZE];

			// cmd系列参数
			std::atomic<std::int64_t> cmd_now_, cmd_end_, cmd_collect_;
		};

		auto tg()->void;
		auto executeCmd(aris::plan::Plan &plan)->int;
		auto checkMotion(const std::uint64_t *mot_options, char *error_msg, std::int64_t count_)->int;
		auto fixError()->std::int32_t;

		Imp(ControlServer *server) :server_(server) {}
		Imp(const Imp&) = delete;

		std::recursive_mutex mu_running_, mu_collect_;
		std::atomic_bool is_running_{ false };

		ControlServer *server_;

		// 主线程ID
		std::thread::id main_thread_id_;

		// mem pool //
		std::vector<char> mempool_;
		
		// chanel //
		enum { CHANEL_SIZE = 4 };
		ChanelData chanels[CHANEL_SIZE];
		int default_chanel_{ 0 };

		// 全局count //
		std::atomic<std::int64_t> global_count_{ 0 };

		// collect系列参数
		std::thread collect_thread_;
		std::atomic_bool is_collect_running_;

		// 储存上一次motion的数据, p v c //
		struct PVC { double p; double v; double c; };
		PVC *last_pvc_, *last_last_pvc_;

		// 交换controller与model所需的缓存 //
		double *mem_transfer_p_, *mem_transfer_v_, *mem_transfer_a_, *mem_transfer_f_;

		// Error 相关
		std::uint64_t *idle_mot_check_options_, *global_mot_check_options_;
		std::atomic<std::int64_t> err_code_and_fixed_{ 0 };
		char err_msg_[1024]{ 0 };
		// error handle //
		std::function<void(aris::plan::Plan *p, int error_num, const char *error_msg)> error_handle_;

		// log 相关
		std::atomic<bool> is_rt_log_started_{ false };

		// 储存Model, Controller, SensorRoot, PlanRoot //
		std::unique_ptr<aris::dynamic::ModelBase> model_;
		std::unique_ptr<aris::control::Controller> controller_;
		std::unique_ptr<aris::control::Master> master_;
		std::unique_ptr<aris::plan::PlanRoot> plan_root_;
		std::unique_ptr<TransferModelController> transfer_model_controller_;
		std::unique_ptr<aris::core::PointerArray<aris::server::Interface>> interface_pool_{new aris::core::PointerArray<aris::server::Interface> };
		std::unique_ptr<MiddleWare> middle_ware_{new MiddleWare};
		std::unique_ptr<CustomModule> custom_module_{new CustomModule};

		// 打洞，读取数据 //
		std::atomic_bool if_get_data_{ false }, if_get_data_ready_{ false };
		const std::function<void(ControlServer&, const aris::plan::Plan *, std::any&)>* get_data_func_;
		std::any *get_data_;

		// callbacks //
		std::atomic<PreCallback> pre_callback_{ nullptr };
		std::atomic<PostCallback> post_callback_{ nullptr };

		// execute in cmd line
		std::vector<std::pair<std::string, std::function<void(aris::plan::Plan&)> > > cmdline_cmd_vec_;
		std::atomic_bool cmdline_msg_received_ = false;
		std::shared_ptr<std::promise<std::vector<std::shared_ptr<aris::plan::Plan>> > > cmdline_execute_promise_;
	};
	auto ControlServer::Imp::tg()->void{
		// pre callback //
		if (auto call = pre_callback_.load())call(ControlServer::instance());

		// 原子操作
		auto global_count = ++global_count_;
		
		union { std::int64_t err_code_and_fixed; struct { std::int32_t code; std::int32_t fix; } err; };
		err_code_and_fixed = err_code_and_fixed_.load();

		// 如果处于错误状态,或者错误还未清理完 //
		if (err_code_and_fixed){
			err.fix = fixError();
			err_code_and_fixed_.store(err_code_and_fixed);
			server_->master().resetRtStasticData(nullptr, false);
			server_->master().lout() << std::flush;
			
			for (int i = 0; i < CHANEL_SIZE; ++i) {
				auto& chanel = chanels[i];
				auto cmd_now = chanel.cmd_now_.load();
				auto cmd_end = chanel.cmd_end_.load();

				// 清理掉所有当前在执行的plan //
				if (cmd_now < cmd_end) {
					auto& p = *chanel.internal_data_queue_[cmd_now % CMD_POOL_SIZE]->plan_;
					p.setExecuteRetCode(err.code);
					p.setExecuteRetMsg(err_msg_);
					for (auto cmd_id = cmd_now + 1; cmd_id < cmd_end; ++cmd_id) {
						auto& p = *chanel.internal_data_queue_[cmd_id % CMD_POOL_SIZE]->plan_;
						p.setExecuteRetCode(aris::plan::Plan::EXECUTE_CANCELLED);
						p.setExecuteRetMsg("execute has been cancelled.");
					}
				}
				chanel.cmd_now_.store(cmd_end);
			}
		}
		// 否则执行cmd queue中的cmd //
		else {
			bool is_idle = true;
			for (int i = 0; i < CHANEL_SIZE; ++i) {
				auto& chanel = chanels[i];
				auto cmd_now = chanel.cmd_now_.load();
				auto cmd_end = chanel.cmd_end_.load();
			
				if (cmd_end > cmd_now) {
					auto& plan = *chanel.internal_data_queue_[cmd_now % CMD_POOL_SIZE]->plan_;

					is_idle = false;

					// 在第一回合初始化，包括log，初始化target等 //
					if (plan.setCount(plan.count() + 1), plan.count() == 1) {
						// 初始化target
						plan.setBeginGlobalCount(global_count);

						// 创建rt_log文件 //
						if (is_rt_log_started_) {
							char name[1000];
							std::sprintf(name, "%" PRId64 "", plan.cmdId());
							server_->master().logFile(name);
						}

						// 初始化统计数据 //
						server_->master().resetRtStasticData(&plan.rtStastic(), true);
					}

					// 执行命令
					auto ret = executeCmd(plan);

					// 错误，包含系统检查出的错误以及用户返回错误 //
					if (((err.code = checkMotion(plan.motorOptions().data(), err_msg_, plan.count())) < 0) || ((err.code = ret) < 0)) {
						err.fix = fixError();
						err_code_and_fixed_.store(err_code_and_fixed);
						if (error_handle_)error_handle_(&plan, err.code, err_msg_);

						// finish //
						if (ret >= 0) { // 只有 plan 认为自己正确的时候，才更改其返回值 //
							plan.setExecuteRetMsg(err_msg_);
							plan.setExecuteRetCode(err.code);
						}
						else {
							std::copy_n(plan.executeRetMsg(), 1024, err_msg_);
							plan.setExecuteRetCode(err.code);
						}
						for (auto cmd_id = cmd_now + 1; cmd_id < cmd_end; ++cmd_id) {
							auto& p = *chanel.internal_data_queue_[cmd_id % CMD_POOL_SIZE]->plan_;
							p.setExecuteRetCode(aris::plan::Plan::EXECUTE_CANCELLED);
							p.setExecuteRetMsg("execute has been cancelled.");
						}

						server_->master().resetRtStasticData(nullptr, false);
						server_->master().lout() << std::flush;
						chanel.cmd_now_.store(cmd_end);// 原子操作
					}
					// 命令正常结束，结束统计数据 //
					else if (ret == 0) {
						// print info //
						if (!(plan.option() & aris::plan::Plan::NOT_PRINT_EXECUTE_COUNT))
							plan.master()->mout() << "RT" << i << " " << plan.cmdId() << "---" 
							<< "cmd finished, spend " << plan.count() << " counts\n";

						// finish //
						server_->master().resetRtStasticData(nullptr, false);
						server_->master().lout() << std::flush;
						chanel.cmd_now_.store(cmd_now + 1);//原子操作
					}
					// 命令仍在执行 //
					else {
						// print info //
						if (plan.count() % 1000 == 0 && !(plan.option() & aris::plan::Plan::NOT_PRINT_EXECUTE_COUNT))
							plan.master()->mout() << "RT" << i << " " << plan.cmdId() << "---" 
							<< "execute cmd in count: " << plan.count() << "\n";
					}
				}
			}
			
			if (is_idle) {
				if (err.code = checkMotion(idle_mot_check_options_, err_msg_, 0); err.code < 0) {
					err.fix = fixError();
					err_code_and_fixed_.store(err_code_and_fixed);
					if (error_handle_)error_handle_(nullptr, err.code, err_msg_);
					server_->master().mout() << "RT  ---failed when idle " << err.code << ":\nRT  ---" << err_msg_ << "\n";
				}
			}
		} 
		

		// 储存本次的数据 //
		for (std::size_t i = 0; i < controller_->motorPool().size(); ++i){
			last_last_pvc_[i].p = controller_->motorPool()[i].targetPos();
			last_last_pvc_[i].v = controller_->motorPool()[i].targetVel();
			last_last_pvc_[i].c = controller_->motorPool()[i].targetToq();
		}
		std::swap(last_pvc_, last_last_pvc_);

		// 给与外部想要的数据 //
		if (if_get_data_.exchange(false)){ // 原子操作
			auto cmd_end = chanels[default_chanel_].cmd_end_.load();
			auto cmd_now = chanels[default_chanel_].cmd_now_.load();
			get_data_func_->operator()(ControlServer::instance(), cmd_end > cmd_now ? &*chanels[0].internal_data_queue_[cmd_now % CMD_POOL_SIZE]->plan_ : nullptr, *get_data_);
			if_get_data_ready_.store(true); // 原子操作
		}

		// post callback //
		if (auto call = post_callback_.load())call(ControlServer::instance());
	}
	auto ControlServer::Imp::executeCmd(aris::plan::Plan &plan)->int{
		// 从controller 向model更新数据 //
		server_->updateDataController2Model(plan.motorOptions());

		// 执行plan函数 //
		int ret = plan.executeRT();

		// 控制电机 //
		server_->updateDataModel2Controller(plan.motorOptions());


		return ret;
	}
	auto ControlServer::Imp::checkMotion(const std::uint64_t *mot_options, char *error_msg, std::int64_t count_)->int{
		int error_code = aris::plan::Plan::SUCCESS;

		// 检查规划的指令是否合理（包括电机是否已经跟随上） //
		for (std::size_t i = 0; i < controller_->motorPool().size(); ++i){
			const auto &cm = controller_->motorPool()[i];
			const auto &ld = last_pvc_[i];
			const auto &lld = last_last_pvc_[i];
			const auto option = mot_options[i];
			const auto dt = master_->samplePeriodNs() / 1.0e9;

			auto display_id = i + 1;

			// 检查使能 //
			if (!(option & aris::plan::Plan::NOT_CHECK_ENABLE)
				&& ((cm.statusWord() & 0x6f) != 0x27))
			{
				error_code = aris::plan::Plan::MOTION_NOT_ENABLED;
				sprintf(error_msg, 
					aris::core::currentLanguage() == (int)aris::core::Language::kSimplifiedChinese ?
					u8"电机 %zd 没在使能模式，当前周期: %zd\n":
					u8"Motor %zd is not in OPERATION_ENABLE mode in count %zd\n",
					display_id, count_);
				return error_code;
			}

			// 使能时才检查 //
			if (cm.isEnabled()){
				switch (cm.modeOfOperation()){
				case 6:break;
				case 8:{
					// check pos infinite //
					if (!std::isfinite(cm.targetPos())){
						error_code = aris::plan::Plan::MOTION_POS_INFINITE;
						sprintf(error_msg,
							aris::core::currentLanguage() == (int)aris::core::Language::kSimplifiedChinese ?
							u8"电机 %zu 目标位置不是有效值，当前周期: %zu\t目标位置: %f\n":
							u8"Motor %zu target position is INFINITE in count %zu:\nvalue: %f\n" ,
							display_id, count_, cm.targetPos());
						return error_code;
					}
					
					// check pos max //
					if (!(option & aris::plan::Plan::NOT_CHECK_POS_MAX)
						&& (cm.targetPos() > cm.maxPos())
						&& (cm.targetPos() > ld.p))
					{
						error_code = aris::plan::Plan::MOTION_POS_BEYOND_MAX;
						sprintf(error_msg, 
							aris::core::currentLanguage() == (int)aris::core::Language::kSimplifiedChinese ?
							u8"电机 %zu 超出正限位，当前周期: %zu\t允许最大位置: %f\t目标位置: %f\n":
							u8"Motor %zu target position beyond MAX in count %zu:\nmax: %f\tnow: %f\n" ,
							display_id, count_, cm.maxPos(), cm.targetPos());
						return error_code;
					}

					// check pos min //
					if (!(option & aris::plan::Plan::NOT_CHECK_POS_MIN)
						&& (cm.targetPos() < cm.minPos())
						&& (cm.targetPos() < ld.p))
					{
						error_code = aris::plan::Plan::MOTION_POS_BEYOND_MIN;
						sprintf(error_msg, 
							aris::core::currentLanguage() == (int)aris::core::Language::kSimplifiedChinese ?
							u8"电机 %zu 超出负限位，当前周期: %zu\t允许最小值: %f\t目标位置: %f\n":
							u8"Motor %zu target position beyond MIN in count %zu:\nmin: %f\tnow: %f\n" ,
							display_id, count_, cm.minPos(), cm.targetPos());
						return error_code;
					}

					// check pos continuous //
					if (!(option & aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS)
						&& ((cm.targetPos() - ld.p) > dt * cm.maxVel() || (cm.targetPos() - ld.p) < dt * cm.minVel()))
					{
						error_code = aris::plan::Plan::MOTION_POS_NOT_CONTINUOUS;
						sprintf(error_msg, 
							aris::core::currentLanguage() == (int)aris::core::Language::kSimplifiedChinese ?
							u8"电机 %zu 速度过大，当前周期: %zu\t上次位置: %f\t本次位置: %f\n":
							u8"Motor %zu target position NOT CONTINUOUS in count %zu:\nlast: %f\tnow: %f\n",
							display_id, count_, ld.p, cm.targetPos());
						return error_code;
					}

					// check pos continuous second order //
					if (!(option & aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER)
						&& ((cm.targetPos() + lld.p - 2 * ld.p) > dt * dt * cm.maxAcc() || (cm.targetPos() + lld.p - 2 * ld.p) < dt * dt * cm.minAcc()))
					{
						error_code = aris::plan::Plan::MOTION_POS_NOT_CONTINUOUS_SECOND_ORDER;
						sprintf(error_msg, 
							aris::core::currentLanguage() == (int)aris::core::Language::kSimplifiedChinese ?
							u8"电机 %zu 加速度过大，当前Count %zu:\n上上次位置: %f\t上次位置: %f\t本次位置: %f\n":
							u8"Motor %zu target position NOT SECOND CONTINUOUS in count %zu:\nlast last: %f\tlast: %f\tnow: %f\n",
							display_id, count_, lld.p, ld.p, cm.targetPos());
						return error_code;
					}

					// check pos following error //
					if (!(option & aris::plan::Plan::NOT_CHECK_POS_FOLLOWING_ERROR)
						&& (std::abs(cm.targetPos() - cm.actualPos()) > cm.maxPosFollowingError()))
					{
						error_code = aris::plan::Plan::MOTION_POS_FOLLOWING_ERROR;
						sprintf(error_msg, 
							aris::core::currentLanguage() == (int)aris::core::Language::kSimplifiedChinese ?
							u8"电机 %zu 位置跟随误差过大，当前Count %zu:\n实际位置: %f\t目标位置: %f\n":
							u8"Motion %zu target position has FOLLOW ERROR in count %zu:\nactual: %f\ttarget: %f\n",
							display_id, count_, cm.actualPos(), cm.targetPos());
						return error_code;
					}

					break;
				}
				case 9:{
					// check vel infinite //
					if (!std::isfinite(cm.targetVel())){
						error_code = aris::plan::Plan::MOTION_VEL_INFINITE;
						sprintf(error_msg, 
							aris::core::currentLanguage() == (int)aris::core::Language::kSimplifiedChinese ?
							u8"电机 %zu 目标速度不是有效值，当前Count %zu:\n目标速度: %f\n":
							u8"Motion %zu target velocity is INFINITE in count %zu:\nvalue: %f\n", 
							display_id, count_, cm.targetVel());
						return error_code;
					}
					
					// check vel max //
					if (!(option & aris::plan::Plan::NOT_CHECK_VEL_MAX)
						&& (cm.targetVel() > cm.maxVel()))
					{
						error_code = aris::plan::Plan::MOTION_VEL_BEYOND_MAX;
						sprintf(error_msg, 
							aris::core::currentLanguage() == (int)aris::core::Language::kSimplifiedChinese ?
							u8"电机 %zu 速度超限，当前Count %zu:\n允许最大速度: %f\t目标速度: %f\n":
							u8"Motion %zu target velocity beyond MAX in count %zu:\nmax: %f\tnow: %f\n", 
							display_id, count_, cm.maxVel(), cm.targetVel());
						return error_code;
					}

					// check vel min //
					if (!(option & aris::plan::Plan::NOT_CHECK_VEL_MIN)
						&& (cm.targetVel() < cm.minVel()))
					{
						error_code = aris::plan::Plan::MOTION_VEL_BEYOND_MIN;
						sprintf(error_msg, 
							aris::core::currentLanguage() == (int)aris::core::Language::kSimplifiedChinese ?
							u8"电机 %zu 速度超限，当前Count %zu:\n允许最小速度: %f\t目标速度: %f\n":
							u8"Motion %zu target velocity beyond MIN in count %zu:\nmin: %f\tnow: %f\n", 
							display_id, count_, cm.minVel(), cm.targetVel());
						return error_code;
					}

					// check vel continuous //
					if (!(option & aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS)
						&& ((cm.targetVel() - ld.v) > dt * cm.maxAcc() || (cm.targetVel() - ld.v) < dt * cm.minAcc()))
					{
						error_code = aris::plan::Plan::MOTION_VEL_NOT_CONTINUOUS;
						sprintf(error_msg, 
							aris::core::currentLanguage() == (int)aris::core::Language::kSimplifiedChinese ?
							u8"电机 %zu 加速度超限，当前Count %zu:\n上次速度: %f\t本次速度: %f\n":
							u8"Motion %zu target velocity NOT CONTINUOUS in count %zu:\nlast: %f\tnow: %f\n", display_id, count_, ld.v, cm.targetVel());
						return error_code;
					}

					// check vel following error //
					if (!(option & aris::plan::Plan::NOT_CHECK_VEL_FOLLOWING_ERROR)
						&& (std::abs(cm.targetVel() - cm.actualVel()) > cm.maxVelFollowingError()))
					{
						error_code = aris::plan::Plan::MOTION_VEL_FOLLOWING_ERROR;
						sprintf(error_msg, 
							aris::core::currentLanguage() == (int)aris::core::Language::kSimplifiedChinese ?
							u8"电机 %zu 速度跟随误差过大，当前Count %zu:\n实际速度: %f\t目标速度: %f\n":
							u8"Motion %zu target velocity has FOLLOW ERROR in count %zu:\nactual: %f\ttarget: %f\n", display_id, count_, cm.actualVel(), cm.targetVel());
						return error_code;
					}

					break;
				}
				case 10:{
					// check pos max //
					if (!(option & aris::plan::Plan::NOT_CHECK_POS_MAX)
						&& (cm.actualPos() > cm.maxPos()))
					{
						error_code = aris::plan::Plan::MOTION_POS_BEYOND_MAX;
						sprintf(error_msg, 
							aris::core::currentLanguage() == (int)aris::core::Language::kSimplifiedChinese ?
							u8"电机 %zu 实际位置超出最大值，当前Count %zu:\n允许最大位置: %f\t实际位置: %f\n" :
							u8"Motion %zu target position beyond MAX in count %zu:\nmax: %f\tnow: %f\n", display_id, count_, cm.maxPos(), cm.targetPos());
						return error_code;
					}

					// check pos min //
					if (!(option & aris::plan::Plan::NOT_CHECK_POS_MIN)
						&& (cm.actualPos() < cm.minPos()))
					{
						error_code = aris::plan::Plan::MOTION_POS_BEYOND_MIN;
						sprintf(error_msg, 
							aris::core::currentLanguage() == (int)aris::core::Language::kSimplifiedChinese ?
							u8"电机 %zu 实际位置超出最小值，当前Count %zu:\n允许最小位置: %f\t实际位置: %f\n" :
							u8"Motion %zu target position beyond MIN in count %zu:\nmin: %f\tnow: %f\n", 
							display_id, count_, cm.minPos(), cm.targetPos());
						return error_code;
					}

					// check vel max //
					if (!(option & aris::plan::Plan::NOT_CHECK_VEL_MAX)
						&& (cm.actualVel() > cm.maxVel()))
					{
						error_code = aris::plan::Plan::MOTION_VEL_BEYOND_MAX;
						sprintf(error_msg, 
							aris::core::currentLanguage() == (int)aris::core::Language::kSimplifiedChinese ?
							u8"电机 %zu 实际速度超出最大值，当前Count %zu:\n允许最大速度: %f\t实际速度: %f\n" :
							u8"Motion %zu target velocity beyond MAX in count %zu:\nmax: %f\tnow: %f\n", 
							display_id, count_, cm.maxVel(), cm.actualVel());
						return error_code;
					}

					// check vel min //
					if (!(option & aris::plan::Plan::NOT_CHECK_VEL_MIN)
						&& (cm.actualVel() < cm.minVel()))
					{
						error_code = aris::plan::Plan::MOTION_VEL_BEYOND_MIN;
						sprintf(error_msg, 
							aris::core::currentLanguage() == (int)aris::core::Language::kSimplifiedChinese ?
							u8"电机 %zu 实际速度超出最小值，当前Count %zu:\n允许最小速度: %f\t实际速度: %f\n" :
							u8"Motion %zu target velocity beyond MIN in count %zu:\nmin: %f\tnow: %f\n", display_id, count_, cm.minVel(), cm.actualVel());
						return error_code;
					}

					// check vel continuous //
					if (!(option & aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS)
						&& ((cm.actualVel() - ld.v) > dt * cm.maxAcc() || (cm.actualVel() - ld.v) < dt * cm.minAcc()))
					{
						error_code = aris::plan::Plan::MOTION_VEL_NOT_CONTINUOUS;
						sprintf(error_msg, 
							aris::core::currentLanguage() == (int)aris::core::Language::kSimplifiedChinese ?
							u8"电机 %zu 加速度过大，当前Count %zu:\n上次速度: %f\t本次速度: %f\n" :
							u8"Motion %zu velocity NOT CONTINUOUS in count %zu:\nlast: %f\tnow: %f\n", 
							display_id, count_, ld.v, cm.actualVel());
						return error_code;
					}
					break;
				}
				default:{
					// invalid mode //
					if (!(option & aris::plan::Plan::NOT_CHECK_MODE)){
						error_code = aris::plan::Plan::MOTION_INVALID_MODE;
						sprintf(error_msg,
							aris::core::currentLanguage() == (int)aris::core::Language::kSimplifiedChinese ?
							u8"电机 %zu 模式不合法，当前Count %zu:\n模式: %d\n" :
							u8"Motion %zu MODE INVALID in count %zu:\nmode: %d\n", 
							display_id, count_, cm.modeOfOperation());
						return error_code;
					}
				}
				}
			}
		}

		
		return 0;
	}
	auto ControlServer::Imp::fixError()->std::int32_t{
		// 只要在check里面执行，都说明修复没有结束 //
		std::int32_t fix_finished{ 0 };
		for (std::size_t i = 0; i < controller_->motorPool().size(); ++i){
			// correct
			auto &cm = controller_->motorPool().at(i);
			switch (cm.modeOfOperation()){
			case 1:
			case 8:
				cm.setTargetPos(std::abs(last_pvc_[i].p - cm.actualPos()) > cm.maxPosFollowingError() ? cm.actualPos() : last_pvc_[i].p);
				break;
			case 9:
				cm.setTargetVel(0.0);
				break;
			case 10:
				cm.setTargetToq(0.0);
				fix_finished = cm.disable() || fix_finished;
				break;
			default:
				fix_finished = cm.disable() || fix_finished;
				cm.setTargetPos(0.0);
				cm.setTargetVel(0.0);
				cm.setTargetToq(0.0);
			}

			// store correct data
			last_pvc_[i].p = last_last_pvc_[i].p = controller_->motorPool().at(i).targetPos();
			last_pvc_[i].v = last_last_pvc_[i].v = controller_->motorPool().at(i).targetVel();
			last_pvc_[i].c = last_last_pvc_[i].c = controller_->motorPool().at(i).targetToq();
		}

		return fix_finished;
	}
	auto ControlServer::instance()noexcept->ControlServer & { static ControlServer instance; return instance; }
	auto ControlServer::resetModel(dynamic::ModelBase *model)->void { imp_->model_.reset(model); }
	auto ControlServer::model()->dynamic::ModelBase& { return *imp_->model_; }

	auto ControlServer::resetMaster(control::Master *master)->void { imp_->master_.reset(master); }
	auto ControlServer::master()->control::Master& { return *imp_->master_; }

	auto ControlServer::resetController(control::Controller *controller)->void{	imp_->controller_.reset(controller);}
	auto ControlServer::controller()->control::Controller& { return *imp_->controller_; }

	auto ControlServer::resetPlanRoot(plan::PlanRoot *plan_root)->void{	imp_->plan_root_.reset(plan_root);}
	auto ControlServer::planRoot()->plan::PlanRoot& { return *imp_->plan_root_; }
	
	auto ControlServer::resetTransferModelController(TransferModelController*method)->void { imp_->transfer_model_controller_.reset(method); }
	auto ControlServer::transferModelController()->TransferModelController& { return *imp_->transfer_model_controller_; }
	
	auto ControlServer::resetInterfacePool(aris::core::PointerArray<aris::server::Interface> *pool)->void {
		imp_->interface_pool_.reset(pool);
	}
	auto ControlServer::interfacePool()->aris::core::PointerArray<aris::server::Interface>& { return *imp_->interface_pool_; }

	auto ControlServer::resetMiddleWare(aris::server::MiddleWare *middle_ware)->void { imp_->middle_ware_.reset(middle_ware); }
	auto ControlServer::middleWare()->MiddleWare& { return *imp_->middle_ware_; }

	auto ControlServer::resetCustomModule(server::CustomModule *custom_module)->void { imp_->custom_module_.reset(custom_module); }
	auto ControlServer::customModule()->CustomModule& { return *imp_->custom_module_; }
	
	auto ControlServer::updateDataController2Model(const std::vector<std::uint64_t>& options)noexcept->void {
		imp_->transfer_model_controller_->updateDataController2Model(options, &controller(), &model());
	}
	auto ControlServer::updateDataModel2Controller(const std::vector<std::uint64_t>& options)noexcept->void {
		imp_->transfer_model_controller_->updateDataModel2Controller(options, &model(), &controller());
	}
	
	auto ControlServer::setRtErrorCallback(std::function<void(aris::plan::Plan *p, int error_num, const char *error_msg)> call_back)noexcept->void {
		imp_->error_handle_ = call_back;
	}
	auto ControlServer::errorMsg()const noexcept->const char * { return imp_->err_msg_; }
	auto ControlServer::setRtPlanPreCallback(PreCallback pre_callback)noexcept->void { imp_->pre_callback_.store(pre_callback); }
	auto ControlServer::setRtPlanPostCallback(PostCallback post_callback)noexcept->void { imp_->post_callback_.store(post_callback); }
	auto ControlServer::running()noexcept->bool { return imp_->is_running_; }
	auto ControlServer::globalCount()noexcept->std::int64_t { return imp_->global_count_.load(); }
	auto ControlServer::currentExecutePlanRt(int chanel)noexcept->aris::plan::Plan *{
		auto cmd_now = imp_->chanels[chanel].cmd_now_.load();
		auto cmd_end = imp_->chanels[chanel].cmd_end_.load();
		return cmd_end > cmd_now ? imp_->chanels[chanel].internal_data_queue_[cmd_now % Imp::CMD_POOL_SIZE]->plan_.get() : nullptr;
	}
	auto ControlServer::globalMotionCheckOption()noexcept->std::uint64_t* { return imp_->global_mot_check_options_; }
	auto ControlServer::idleMotionCheckOption()noexcept->std::uint64_t* { return imp_->idle_mot_check_options_; }
	auto ControlServer::setAutoLogActive(bool auto_log)noexcept->void { imp_->is_rt_log_started_.store(auto_log); }
	auto ControlServer::autoLogActive()noexcept->bool { return imp_->is_rt_log_started_.load(); }

	auto ControlServer::open()->void{ for (auto &inter : interfacePool()) inter.open();	}
	auto ControlServer::close()->void { for (auto &inter : interfacePool()) inter.close(); }
	auto ControlServer::runCmdLine()->void{
		static TerminalInterface terminal;

		auto ret = std::async(std::launch::async, []()->std::string{
			std::string command_in;
			std::getline(std::cin, command_in);
			if (command_in.empty())std::this_thread::sleep_for(std::chrono::milliseconds(1));
			return command_in;
		});

		imp_->main_thread_id_ = std::this_thread::get_id();

		for (;;){
			// 检测是否有数据从executeCmdInMain过来
			if (imp_->cmdline_msg_received_){
				auto ret_plan = executeCmd(imp_->cmdline_cmd_vec_);
				imp_->cmdline_msg_received_ = false;
				imp_->cmdline_execute_promise_->set_value(ret_plan);
			}
			// 检测是否有数据从command line过来
			else if (ret.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready){
				// 在linux后台可能getline失败，得到空字符串 //
				if (auto cmd_str = ret.get(); !cmd_str.empty())	{
					imp_->middle_ware_->executeCmd(cmd_str, dynamic_cast<Interface*>(&terminal));
				}

				ret = std::async(std::launch::async, []()->std::string{
					std::string command_in;
					std::getline(std::cin, command_in);
					if (command_in.empty())std::this_thread::sleep_for(std::chrono::milliseconds(1));
					return command_in;
				});
			}
			// 休息
			else{
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
			}
		}
	}
	auto ControlServer::setDefaultChanel(int chanel)noexcept->void {
		imp_->default_chanel_ = chanel;
	}
	auto ControlServer::defaultChanel()const noexcept->int {
		return imp_->default_chanel_;
	}
	auto ControlServer::executeCmd(std::vector<std::pair<std::string, std::function<void(aris::plan::Plan&)> > > cmd_vec, int chanel)->std::vector<std::shared_ptr<aris::plan::Plan>>{
		std::unique_lock<std::recursive_mutex> running_lck(imp_->mu_running_);

		auto setRetCodeAndOption = [](const std::shared_ptr<aris::plan::Plan>& plan, std::int32_t code)->void {
			plan->setPrepareRetCode(code);
			plan->option() |= aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
		};

		// pre process //
		chanel = chanel == -1 ? defaultChanel() : chanel;

		// step 1.  parse //
		std::vector<std::shared_ptr<Imp::InternalData>> internal_data(cmd_vec.size());
		std::vector<std::shared_ptr<aris::plan::Plan>> ret_plan(cmd_vec.size());
		for (aris::Size i = 0; i < cmd_vec.size(); ++i) {
			auto& str           = cmd_vec[i].first;
			auto& post_callback = cmd_vec[i].second;
			
			internal_data[i] = std::shared_ptr<Imp::InternalData>(new Imp::InternalData{ std::shared_ptr<aris::plan::Plan>(nullptr), post_callback });
			
			auto &plan = internal_data[i]->plan_;
			try{ // case 1.1 : success
				std::vector<char> cmd_str_local(str.size());
				std::copy(str.begin(), str.end(), cmd_str_local.begin());

				static std::uint64_t cmd_id{ 0 };
				++cmd_id;
				ARIS_LOG(aris::core::LogLvl::kDebug, 0, { "server parse cmd %ji : %s", "服务器分析指令 %ji : %s" }, cmd_id, str.data());
				auto[cmd, params] = planRoot().planParser().parse(std::string_view(cmd_str_local.data(), cmd_str_local.size())); // may throw
				auto plan_iter = std::find_if(planRoot().planPool().begin(), planRoot().planPool().end(), [&](const plan::Plan &p) {return p.command().name() == cmd; });
				plan = std::shared_ptr<aris::plan::Plan>(dynamic_cast<aris::plan::Plan*>(plan_iter->clone()));
				ret_plan[i] = plan;
				plan->setSharedPtrForThis(plan);

				plan->setCmdId(cmd_id);
				plan->setCount(0);
				plan->setBeginGlobalCount(0);
				
				plan->setControlServer(this);
				plan->setModelBase(imp_->model_.get());
				plan->setMaster(imp_->master_.get());
				plan->setController(imp_->controller_.get());
				
				plan->option() = 0;
				plan->motorOptions().resize(plan->controller()->motorPool().size(), 0);
				std::copy_n(imp_->global_mot_check_options_, plan->controller()->motorPool().size(), plan->motorOptions().data());
				
				plan->rtStastic() = aris::control::Master::RtStasticsData{ 0,0,0,0x8fffffff,0,0,0 };
				plan->setPrepareRetCode(aris::plan::Plan::SUCCESS);
				std::fill_n(plan->prepareRetMsg(), 1024, '\0');
				plan->setExecuteRetCode(aris::plan::Plan::SUCCESS);
				std::fill_n(plan->executeRetMsg(), 1024, '\0');

				plan->command().init();
				plan->parse(str); // may throw, set cmd_string cmd_name & cmd_params 
			}
			catch (std::exception &e){ // case 1.2 : exception
				for (aris::Size j = 0; j < i; j++)
					setRetCodeAndOption(ret_plan[j], aris::plan::Plan::PREPARE_CANCELLED);
				for (aris::Size j = i + 1; j < cmd_vec.size(); j++) {
					internal_data[j] = std::shared_ptr<Imp::InternalData>(new Imp::InternalData{ std::shared_ptr<aris::plan::Plan>(new aris::plan::Plan), cmd_vec[j].second });
					ret_plan[j] = internal_data[j]->plan_;
					setRetCodeAndOption(ret_plan[j], aris::plan::Plan::PREPARE_CANCELLED);
				}
				
				internal_data[i]->plan_ =  std::shared_ptr<aris::plan::Plan>(new aris::plan::Plan);
				ret_plan[i] = internal_data[i]->plan_;
				setRetCodeAndOption(ret_plan[i], aris::plan::Plan::PREPARE_CANCELLED);
				std::copy_n(e.what(), std::strlen(e.what()), ret_plan[i]->prepareRetMsg());
				return ret_plan;
			}
		}

		// step 2.  prepare //
		bool prepare_error = false;
		for (auto p = internal_data.begin(); p < internal_data.end(); ++p){
			auto &plan = (*p)->plan_;
			try	{
				ARIS_LOG(aris::core::LogLvl::kDebug, 0, { "server prepare cmd %ji" }, plan->cmdId());
				plan->prepareNrt();
				(*p)->has_prepared_ = true;

				// false : case 2.1    true : case 2.2 
				if (plan->prepareRetCode() < 0) { 
					for (auto pp = internal_data.begin(); pp < internal_data.end(); ++pp) {
						if (pp < p) setRetCodeAndOption((*pp)->plan_, aris::plan::Plan::EXECUTE_CANCELLED);
						if (pp > p) setRetCodeAndOption((*pp)->plan_, aris::plan::Plan::PREPARE_CANCELLED);
					}
					plan->option() |= aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
					prepare_error = true;
					break;
				}
			}
			catch (std::exception &e){ // case 2.3
				for (auto pp = internal_data.begin(); pp < internal_data.end(); ++pp){
					if (pp < p) setRetCodeAndOption((*pp)->plan_, aris::plan::Plan::EXECUTE_CANCELLED);
					if (pp > p) setRetCodeAndOption((*pp)->plan_, aris::plan::Plan::PREPARE_CANCELLED);
				}
				setRetCodeAndOption(plan, aris::plan::Plan::PREPARE_EXCEPTION);
				std::copy_n(e.what(), std::strlen(e.what()), plan->prepareRetMsg());
				prepare_error = true;
				break;
			}
		}

		// print and log cmd info /////////////////////////////////////////////////////////////////////////////////////////////////////////////
		for (auto &plan : ret_plan){
			auto print_size = plan->cmdParams().empty() ? 2 : 2 + std::max_element(plan->cmdParams().begin(), plan->cmdParams().end(), [](const auto& a, const auto& b){
				return a.first.length() < b.first.length();
			})->first.length();
			// print
			if (!(plan->option() & aris::plan::Plan::NOT_PRINT_CMD_INFO)){
				ARIS_COUT << "cmd " << plan->cmdId() << "---" << plan->cmdString() << "\n";
				ARIS_COUT_PLAN(plan) << plan->cmdName() << "\n";
				for (auto &p : plan->cmdParams())ARIS_COUT_PLAN(plan) << std::string(print_size - p.first.length(), ' ') << p.first << " : " << p.second << "\n";
				ARIS_COUT << std::endl;
			}
			// log
			if (!(plan->option() & aris::plan::Plan::NOT_LOG_CMD_INFO)){
				std::stringstream ss;
				ss << plan->cmdName() << "\n";
				for (auto &p : plan->cmdParams()){
					ss << std::string(print_size - p.first.length(), ' ') << p.first << " : " << p.second << std::endl;
				}
				ARIS_LOG(aris::core::LogLvl::kDebug, 0, { ss.str().c_str() });
			}
		}
		// print over ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		if(prepare_error)return ret_plan;

		// step 3.  execute //
		{
			// 构造需要 execute 的 cmd_list //
			std::list<std::shared_ptr<Imp::InternalData>> need_run_internal;
			for (auto& inter : internal_data)
				if (!(inter->plan_->option() & aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION))
					need_run_internal.push_back(inter);

			// 检查 server 是否已经在运行 //
			if (!imp_->is_running_) {
				for (auto& inter : need_run_internal) {
					setRetCodeAndOption(inter->plan_, aris::plan::Plan::SERVER_NOT_STARTED);
					inter->plan_->setPrepareRetMsg("server not started, use cs_start to start");
				}
				return ret_plan;
			}

			// 检查 server 是否处于错误状态 //
			if (auto err = this->errorCode()) {
				for (auto& inter : need_run_internal) {
					setRetCodeAndOption(inter->plan_, err);
					inter->plan_->setPrepareRetMsg("server in error, use cl to clear");
				}
				return ret_plan;
			}

			// 查看是否 plan 池已满 //
			auto cmd_end = imp_->chanels[chanel].cmd_end_.load();
			if ((cmd_end - imp_->chanels[chanel].cmd_collect_.load() + need_run_internal.size()) >= Imp::CMD_POOL_SIZE) {//原子操作(cmd_now)
				for (auto& inter : need_run_internal) {
					setRetCodeAndOption(inter->plan_, aris::plan::Plan::COMMAND_POOL_IS_FULL);
					inter->plan_->setPrepareRetMsg("command pool is full");
				}
				return ret_plan;
			}

			// 添加命令 //
			for (auto& inter : need_run_internal) {
				imp_->chanels[chanel].internal_data_queue_[cmd_end++ % Imp::CMD_POOL_SIZE] = inter;
				ARIS_LOG(aris::core::LogLvl::kDebug, 0, { "server execute cmd %ji" }, inter->plan_->cmdId());
			}
			imp_->chanels[chanel].cmd_end_.store(cmd_end);
		}

		// step 4.   in RT

		// step 5&6. USE RAII //
		return ret_plan;
	}
	auto ControlServer::executeCmd(std::string cmd_str, std::function<void(aris::plan::Plan&)> post_callback, int chanel)->std::shared_ptr<aris::plan::Plan>{
		std::vector<std::pair<std::string, std::function<void(aris::plan::Plan&)> > > cmd_vec{ std::make_pair(cmd_str, post_callback) };
		auto ret = executeCmd(cmd_vec, chanel);
		return ret.front();
	}
	auto ControlServer::executeCmdInCmdLine(std::vector<std::pair<std::string, std::function<void(aris::plan::Plan&)>>> cmd_vec, int chanel)->std::vector<std::shared_ptr<aris::plan::Plan>>{
		static std::mutex mu_;
		std::unique_lock<std::mutex> lck(mu_);

		const auto &cur_thread_id = std::this_thread::get_id();
		if (cur_thread_id == imp_->main_thread_id_) {
			return executeCmd(cmd_vec, chanel);
		} 
		else {
			imp_->cmdline_execute_promise_ = std::make_shared<std::promise<std::vector<std::shared_ptr<aris::plan::Plan>>>>();
			auto ret = imp_->cmdline_execute_promise_->get_future();
			imp_->cmdline_cmd_vec_ = cmd_vec;
			imp_->cmdline_msg_received_ = true;

			return ret.get();
		}
	}
	auto ControlServer::executeCmdInCmdLine(std::string cmd_string, std::function<void(aris::plan::Plan&)> post_callback, int chanel)->std::shared_ptr<aris::plan::Plan>{
		std::vector<std::pair<std::string, std::function<void(aris::plan::Plan&)> > > cmd_vec{ std::make_pair(cmd_string, post_callback) };
		auto ret = executeCmdInCmdLine(cmd_vec, chanel);
		return ret.front();
	}
	auto ControlServer::init()->void{
		model().init();
		master().init();
		controller().init();
		planRoot().init();
		middleWare().init();

		// 更新每个 plan 的初值 //
		for (auto &p : planRoot().planPool()) {
			p.setCmdId(0);
			p.setCount(0);
			p.setBeginGlobalCount(0);

			p.setControlServer(this);
			p.setModelBase(imp_->model_.get());
			p.setMaster(imp_->master_.get());
			p.setController(imp_->controller_.get());

			p.option() = 0;
			p.motorOptions().resize(p.controller()->motorPool().size(), 0);

			p.rtStastic() = aris::control::Master::RtStasticsData{ 0,0,0,0x8fffffff,0,0,0 };
			p.setPrepareRetCode(aris::plan::Plan::SUCCESS);
			std::fill_n(p.prepareRetMsg(), 1024, '\0');

			p.setExecuteRetCode(aris::plan::Plan::SUCCESS);
			std::fill_n(p.executeRetMsg(), 1024, '\0');
		}

		// 分配自身所需要的内存 //
		Size mem_size = 0;

		core::allocMem(mem_size, imp_->last_pvc_, controller().motorPool().size());
		core::allocMem(mem_size, imp_->last_last_pvc_, controller().motorPool().size());
		core::allocMem(mem_size, imp_->idle_mot_check_options_, controller().motorPool().size());
		core::allocMem(mem_size, imp_->global_mot_check_options_, controller().motorPool().size());

		core::allocMem(mem_size, imp_->mem_transfer_p_, imp_->model_->inputPosSize());
		core::allocMem(mem_size, imp_->mem_transfer_v_, imp_->model_->inputVelSize());
		core::allocMem(mem_size, imp_->mem_transfer_a_, imp_->model_->inputAccSize());
		core::allocMem(mem_size, imp_->mem_transfer_f_, imp_->model_->inputFceSize());

		imp_->mempool_.resize(mem_size, char(0));

		imp_->last_pvc_ = core::getMem(imp_->mempool_.data(), imp_->last_pvc_);
		imp_->last_last_pvc_ = core::getMem(imp_->mempool_.data(), imp_->last_last_pvc_);
		imp_->idle_mot_check_options_ = core::getMem(imp_->mempool_.data(), imp_->idle_mot_check_options_);
		std::fill_n(imp_->idle_mot_check_options_, controller().motorPool().size(), aris::plan::Plan::NOT_CHECK_ENABLE | aris::plan::Plan::NOT_CHECK_POS_MAX | aris::plan::Plan::NOT_CHECK_POS_MIN);
		imp_->global_mot_check_options_ = core::getMem(imp_->mempool_.data(), imp_->global_mot_check_options_);
		std::fill_n(imp_->global_mot_check_options_, controller().motorPool().size(), std::uint64_t(0));

		imp_->mem_transfer_p_ = core::getMem(imp_->mempool_.data(), imp_->mem_transfer_p_);
		std::fill_n(imp_->mem_transfer_p_, model().inputPosSize(), 0.0);
		imp_->mem_transfer_v_ = core::getMem(imp_->mempool_.data(), imp_->mem_transfer_v_);
		std::fill_n(imp_->mem_transfer_v_, model().inputVelSize(), 0.0);
		imp_->mem_transfer_a_ = core::getMem(imp_->mempool_.data(), imp_->mem_transfer_a_);
		std::fill_n(imp_->mem_transfer_a_, model().inputAccSize(), 0.0);
		imp_->mem_transfer_f_ = core::getMem(imp_->mempool_.data(), imp_->mem_transfer_f_);
		std::fill_n(imp_->mem_transfer_f_, model().inputFceSize(), 0.0);

		// 赋予初值 //
		master().setControlStrategy([this]() {this->imp_->tg(); }); // controller可能被reset，因此这里必须重新设置//

		for (int i = 0; i < Imp::CHANEL_SIZE; ++i) {
			imp_->chanels[i].cmd_now_.store(0);
			imp_->chanels[i].cmd_end_.store(0);
			imp_->chanels[i].cmd_collect_.store(0);
		}

	}
	auto ControlServer::start()->void{
		std::unique_lock<std::recursive_mutex> running_lck(imp_->mu_running_);
		if (imp_->is_running_)THROW_FILE_LINE("failed to start server, because it is already started ");
		
		struct RaiiCollector{
			ControlServer *cs_;
			auto reset()->void { cs_ = nullptr; }
			RaiiCollector(ControlServer *cs) :cs_(cs) {}
			~RaiiCollector(){
				if (cs_){
					cs_->imp_->is_running_ = false;
					cs_->imp_->is_collect_running_ = false;
					if (cs_->imp_->collect_thread_.joinable())cs_->imp_->collect_thread_.join();
				}
			}
		};
		RaiiCollector raii_collector(this);

		imp_->is_running_ = true;

		// start collect thread //
		imp_->is_collect_running_ = true;
		imp_->collect_thread_ = std::thread([this](){
			while (this->imp_->is_collect_running_){
				bool has_collect_plan = false;
				for (int i = 0; i < Imp::CHANEL_SIZE; ++i) {
					auto cmd_collect = imp_->chanels[i].cmd_collect_.load();//原子操作
					auto cmd_now = imp_->chanels[i].cmd_now_.load();//原子操作

					// step 4b. //
					if (cmd_collect < cmd_now) {
						has_collect_plan = true;

						auto& internal_data = imp_->chanels[i].internal_data_queue_[cmd_collect % Imp::CMD_POOL_SIZE];
						auto& plan = *internal_data->plan_;

						// make rt stastic thread safe //
						while (globalCount() == plan.beginGlobalCount() + plan.count() - 1) { std::this_thread::sleep_for(std::chrono::milliseconds(1)); }

						std::stringstream ss;
						ss << "cmd " << plan.cmdId() << " stastics:" << std::endl
							<< std::setw(20) << "avg time(ns):" << std::int64_t(plan.rtStastic().avg_time_consumed) << std::endl
							<< std::setw(20) << "max time(ns):" << plan.rtStastic().max_time_consumed << std::endl
							<< std::setw(20) << "in count:" << plan.rtStastic().max_time_occur_count << std::endl
							<< std::setw(20) << "min time(ns):" << plan.rtStastic().min_time_consumed << std::endl
							<< std::setw(20) << "in count:" << plan.rtStastic().min_time_occur_count << std::endl
							<< std::setw(20) << "total count:" << plan.rtStastic().total_count << std::endl
							<< std::setw(20) << "overruns:" << plan.rtStastic().overrun_count << std::endl;

						ARIS_LOG(aris::core::LogLvl::kDebug, 0, { ss.str().data() });

						// step 4b&5b //
						internal_data.reset();
						aris::server::ControlServer::instance().imp_->chanels[i].cmd_collect_++;
					}
				}
			
				if (!has_collect_plan) {
					std::this_thread::sleep_for(std::chrono::milliseconds(1));
				}
			}
		});

		master().start();

		raii_collector.reset();
	}
	auto ControlServer::stop()->void
	{
		std::unique_lock<std::recursive_mutex> running_lck(imp_->mu_running_);
		if (!imp_->is_running_)THROW_FILE_LINE("failed to stop server, because it is not running");
		imp_->is_running_ = false;

		// 清除所有指令，并回收所有指令 //
		for (int i = 0; i < Imp::CHANEL_SIZE; ++i) {
			imp_->chanels[i].cmd_now_.store(imp_->chanels[i].cmd_end_.load());
			while (imp_->chanels[i].cmd_collect_.load() < imp_->chanels[i].cmd_end_.load()) { std::this_thread::yield(); }
		}
		imp_->is_collect_running_ = false;
		imp_->collect_thread_.join();

		// 停止控制器 //
		master().stop();
	}
	auto ControlServer::waitForAllExecution(int chanel)->void {
		// pre process //
		chanel = chanel == -1 ? defaultChanel() : chanel;
		while (imp_->chanels[chanel].cmd_end_.load() != imp_->chanels[chanel].cmd_now_.load())std::this_thread::sleep_for(std::chrono::milliseconds(1));//原子操作
	}
	auto ControlServer::waitForAllCollection(int chanel)->void {
		// pre process //
		chanel = chanel == -1 ? defaultChanel() : chanel;
		while (imp_->chanels[chanel].cmd_end_.load() != imp_->chanels[chanel].cmd_collect_.load()) std::this_thread::sleep_for(std::chrono::milliseconds(1));//原子操作
	}
	auto ControlServer::currentExecutePlan(int chanel)->std::shared_ptr<aris::plan::Plan>{
		// pre process //
		chanel = chanel == -1 ? defaultChanel() : chanel;

		std::unique_lock<std::recursive_mutex> running_lck(imp_->mu_collect_);
		if (!imp_->is_running_)
			THROW_FILE_LINE("failed to get current TARGET, because ControlServer is not running");

		auto execute_internal = imp_->chanels[chanel].internal_data_queue_[imp_->chanels[chanel].cmd_now_.load() % Imp::CMD_POOL_SIZE];
		return execute_internal ? execute_internal->plan_ : std::shared_ptr<aris::plan::Plan>();
	}
	auto ControlServer::getRtData(const std::function<void(ControlServer&, const aris::plan::Plan *, std::any&)>& get_func, std::any& data)->void
	{
		std::unique_lock<std::recursive_mutex> running_lck(imp_->mu_running_);
		if (!imp_->is_running_)
			THROW_FILE_LINE("failed getRtData");

		imp_->get_data_func_ = &get_func;
		imp_->get_data_ = &data;

		imp_->if_get_data_ready_.store(false);
		imp_->if_get_data_.store(true);

		while (!imp_->if_get_data_ready_.load()) std::this_thread::sleep_for(std::chrono::milliseconds(1));

		imp_->if_get_data_ready_.store(false);
	}
	auto ControlServer::setErrorCode(std::int32_t err_code, const char* err_msg)noexcept->void {
		union { std::int64_t err_code_and_fixed; struct { std::int32_t code; std::int32_t fix; } err; };
		err.code = err_code;
		imp_->err_code_and_fixed_.store(err_code_and_fixed);
		if (err_msg)std::strcpy(imp_->err_msg_, err_msg);
	}
	auto ControlServer::errorCode()const noexcept->int {
		union { std::int64_t err_code_and_fixed; struct { std::int32_t err_code; std::int32_t is_fixed; } err; };
		err_code_and_fixed = imp_->err_code_and_fixed_.load();
		return err.err_code;
	}
	auto ControlServer::clearError()->void 
	{ 
		std::unique_lock<std::recursive_mutex> running_lck(imp_->mu_running_);
		if (!imp_->is_running_)
		{
			imp_->err_code_and_fixed_.store(0);
			std::fill_n(imp_->err_msg_, 1024, '\0');
		}
		else
		{
			//while (imp_->err_code_and_fixed_.load())
			//{
			//	union { std::int64_t err_code_and_fixed; struct { std::int32_t err_code; std::int32_t is_fixed; } err; };
			//	err.err_code = 0;
			//	err.is_fixed = 0xFFFF'FFFF;
			//	imp_->err_code_and_fixed_ &= err_code_and_fixed;
			//	std::this_thread::sleep_for(std::chrono::nanoseconds(master().samplePeriodNs()));
			//}

			// 本函数只负责清理code标志位，fix标志位系统内部会自行清理掉 //
			union { std::int64_t err_code_and_fixed; struct { std::int32_t err_code; std::int32_t is_fixed; } err; };
			err.err_code = 0;
			err.is_fixed = 0xFFFF'FFFF;
			imp_->err_code_and_fixed_ &= err_code_and_fixed;
			std::fill_n(imp_->err_msg_, 1024, '\0');
		}
	}
	ControlServer::~ControlServer() { 
		close();
		if(running())stop();
	}
	ControlServer::ControlServer() :imp_(new Imp(this))	{
		// create members //
		makeModel<aris::dynamic::Model>();
		makeMaster<aris::control::Master>();
		makeController<aris::control::Controller>("controller");
		makePlanRoot<aris::plan::PlanRoot>("plan_root");
		makeTransferModelController<TransferModelController>();
	}

#define ARIS_PRO_COUT ARIS_COUT << "pro "
	struct ProgramMiddleware::Imp{
		std::unique_ptr<aris::core::Socket> sock_{ new aris::core::Socket };

		aris::core::CommandParser command_parser_;
		aris::core::LanguageParser language_parser_;
		aris::core::Calculator calculator_;
		std::thread auto_thread_;
		std::mutex auto_mu_;

		int current_line_{ 0 };
		std::string current_file_;
		bool is_auto_mode_{ false };

		std::string last_error_;
		int last_error_code_{ 0 }, last_error_line_{ 0 };

		std::atomic_bool is_stop_{ false }, is_pause_{ false };

	};
	auto ProgramMiddleware::lastError()->std::string { return imp_->last_error_; }
	auto ProgramMiddleware::lastErrorCode()->int { return imp_->last_error_code_; }
	auto ProgramMiddleware::lastErrorLine()->int { return imp_->last_error_line_; }
	auto ProgramMiddleware::isAutoMode()->bool { return imp_->is_auto_mode_; }
	auto ProgramMiddleware::isAutoRunning()->bool { return imp_->auto_thread_.joinable(); }
	auto ProgramMiddleware::isAutoPaused()->bool { return imp_->is_pause_.load(); }
	auto ProgramMiddleware::isAutoStopped()->bool { return imp_->is_stop_.load(); }
	auto ProgramMiddleware::currentFileLine()->std::tuple<std::string, int>	{
		std::unique_lock<std::mutex> lck(imp_->auto_mu_);
		return std::make_tuple(imp_->current_file_, imp_->current_line_);
	}
	auto ProgramMiddleware::executeCmd(std::string_view str, std::function<void(std::string)> send_ret, Interface *interface)->int
	{
		(void)(interface);
		auto send_code_and_msg = [send_ret](int code, const std::string& ret_msg_str)->int
		{
			nlohmann::json js;
			js["return_code"] = code;///////////////////////////////////////////////////////////
			js["return_message"] = ret_msg_str;

			auto ret_str = js.dump(-1,' ',true);

			ARIS_PRO_COUT << "---" << ret_str << std::endl;

			send_ret(ret_str);
			return 0;
		};

		std::string_view cmd;
		std::map<std::string_view, std::string_view> params;
		try { std::tie(cmd, params) = imp_->command_parser_.parse(str); }
		catch (std::exception &) {};

		if (cmd == "program"){
			ARIS_PRO_COUT << "---" << str << std::endl;
			ARIS_LOG(aris::core::LogLvl::kInfo, 0, { "pro ---%s" }, str.data());

			for (auto &[param, value] : params){
				if (param == "set_auto"){
					imp_->is_auto_mode_ = true;
					return send_code_and_msg(0, "");
				}
				else if (param == "set_manual")	{
					if (isAutoRunning()){
						return send_code_and_msg(aris::plan::Plan::PROGRAM_EXCEPTION, "can not set manual when auto running");
					}
					else{
						imp_->is_auto_mode_ = false;
						return send_code_and_msg(0, "");
					}
				}
				else if (param == "content")
				{
					if (isAutoRunning()){
						return send_code_and_msg(aris::plan::Plan::PROGRAM_EXCEPTION, "can not set content when auto running");
					}
					else{
						imp_->last_error_.clear();
						imp_->last_error_code_ = 0;
						imp_->last_error_line_ = 0;

						auto begin_pos = value.find("{");
						auto end_pos = value.rfind("}");
						auto cmd_str = value.substr(begin_pos + 1, end_pos - 1 - begin_pos);

						try
						{
							//imp_->calculator_ = aris::server::ControlServer::instance().model().calculator();
							auto &c = imp_->calculator_;

							auto js = nlohmann::json::parse(cmd_str);

							std::cout << js << std::endl;

							std::map<std::string, std::string> files;
							for (auto &node : js){
								files[node["name"].get<std::string>()] = node["content"].get<std::string>();
								std::cout << node["content"].get<std::string>() << std::endl;
							}

							imp_->language_parser_.setProgram(files);
							imp_->language_parser_.parseLanguage();

							for (auto &str : imp_->language_parser_.varPool())
							{
								auto cut_str = [](std::string_view &input, const char *c)->std::string_view
								{
									// 此时c中字符是或的关系 //
									auto point = input.find_first_of(c);
									auto ret = input.substr(0, point);
									input = point == std::string::npos ? std::string_view() : input.substr(point);
									return ret;
								};
								auto trim_left = [](std::string_view &input, const char *c)->std::string_view
								{
									auto point = input.find_first_not_of(c);
									return point == std::string::npos ? std::string_view() : input.substr(point, std::string::npos);
								};

								std::string_view input = str;
								if (auto var = cut_str(input, " "); var.empty())THROW_FILE_LINE("invalid command string: please at least contain a word");
								input = trim_left(input, " ");

								auto type = cut_str(input, " ");
								input = trim_left(input, " ");

								auto name = cut_str(input, " =");
								input = trim_left(input, " =");

								auto value = input;
								c.addVariable(name, type, c.calculateExpression(std::string(type) + "(" + std::string(value) + ")").second);
							}

							return send_code_and_msg(0, std::string());
						}
						catch (std::exception &e)
						{
							ARIS_COUT << e.what() << std::endl;
							ARIS_LOG(aris::core::LogLvl::kError, 0, { "pro ---%s" }, e.what());
							return send_code_and_msg(aris::plan::Plan::PROGRAM_EXCEPTION, e.what());
						}
					}
				}
				else if (param == "goto")
				{
					if (!isAutoMode())
					{
						return send_code_and_msg(aris::plan::Plan::PROGRAM_EXCEPTION, "can not goto in manual mode");
					}
					else if (isAutoRunning())
					{
						return send_code_and_msg(aris::plan::Plan::PROGRAM_EXCEPTION, "can not goto when running");
					}
					else
					{
						try
						{
							auto file = value.substr(0, value.find_first_of("."));
							auto line = std::stoi(std::string(value.substr(value.find_first_of(".") + 1)));

							imp_->language_parser_.gotoFileLine(std::string(file) + ".aris", line);
							std::unique_lock<std::mutex> lck(this->imp_->auto_mu_);
							imp_->current_line_ = imp_->language_parser_.currentLine();
							imp_->current_file_ = imp_->language_parser_.currentFile();
							return send_code_and_msg(0, "");
						}
						catch (std::exception &e)
						{
							return send_code_and_msg(-1, e.what());
						}
					}
				}
				else if (param == "goto_main")
				{
					if (!isAutoMode())
					{
						return send_code_and_msg(aris::plan::Plan::PROGRAM_EXCEPTION, "can not goto in manual mode");
					}
					else if (isAutoRunning())
					{
						return send_code_and_msg(aris::plan::Plan::PROGRAM_EXCEPTION, "can not goto when running");
					}
					else
					{
						imp_->language_parser_.gotoMain();
						std::unique_lock<std::mutex> lck(this->imp_->auto_mu_);
						imp_->current_line_ = imp_->language_parser_.currentLine();
						imp_->current_file_ = imp_->language_parser_.currentFile();
						return send_code_and_msg(0, "");
					}
				}
				else if (param == "forward")
				{
					if (!isAutoMode())
					{
						return send_code_and_msg(aris::plan::Plan::PROGRAM_EXCEPTION, "can not foward in manual mode");
					}
					else if (isAutoRunning())
					{
						return send_code_and_msg(aris::plan::Plan::PROGRAM_EXCEPTION, "can not foward when running");
					}
					else if (lastErrorCode())
					{
						return send_code_and_msg(lastErrorCode(), lastError());
					}
					else
					{
						std::swap(imp_->calculator_, dynamic_cast<aris::dynamic::Model&>(aris::server::ControlServer::instance().model()).calculator());
						auto &c = dynamic_cast<aris::dynamic::Model&>(aris::server::ControlServer::instance().model()).calculator();
						auto &cs = aris::server::ControlServer::instance();

						if (imp_->language_parser_.isEnd())
						{

						}
						else if (imp_->language_parser_.isCurrentLineKeyWord())
						{
							ARIS_PRO_COUT << imp_->language_parser_.currentLine() << "---" << imp_->language_parser_.currentCmd() << std::endl;
							//LOG_INFO << "pro " << imp_->language_parser_.currentLine() << "---" << imp_->language_parser_.currentCmd() << std::endl;
							if (imp_->language_parser_.currentWord() == "if" || imp_->language_parser_.currentWord() == "while")
							{
								try
								{
									auto ret = c.calculateExpression(imp_->language_parser_.currentParamStr());

									if (auto ret_double = std::any_cast<double>(&ret.second))
									{
										imp_->language_parser_.forward(*ret_double != 0.0);
										std::unique_lock<std::mutex> lck(this->imp_->auto_mu_);
										imp_->current_line_ = imp_->language_parser_.currentLine();
									}
									else if (auto ret_mat = std::any_cast<aris::core::Matrix>(&ret.second))
									{
										imp_->language_parser_.forward(ret_mat->toDouble() != 0.0);
										std::unique_lock<std::mutex> lck(this->imp_->auto_mu_);
										imp_->current_line_ = imp_->language_parser_.currentLine();
									}
									else
									{
										imp_->last_error_code_ = aris::plan::Plan::PROGRAM_EXCEPTION;
										imp_->last_error_ = "invalid expresion";
										imp_->last_error_line_ = imp_->language_parser_.currentLine();
										ARIS_PRO_COUT << imp_->last_error_line_ << "---err_code:" << imp_->last_error_code_ << "  err_msg:" << imp_->last_error_ << std::endl;
										break;
									}
								}
								catch (std::exception &e)
								{
									imp_->last_error_code_ = -10;
									imp_->last_error_ = e.what();
									imp_->last_error_line_ = imp_->language_parser_.currentLine();
									ARIS_PRO_COUT << imp_->last_error_line_ << "---err_code:" << imp_->last_error_code_ << "  err_msg:" << imp_->last_error_ << std::endl;
									break;
								}
							}
							else
							{
								imp_->language_parser_.forward();
								std::unique_lock<std::mutex> lck(this->imp_->auto_mu_);
								imp_->current_line_ = imp_->language_parser_.currentLine();
							}
						}
						else if (imp_->language_parser_.isCurrentLineFunction())
						{
							ARIS_PRO_COUT << imp_->language_parser_.currentLine() << "---" << imp_->language_parser_.currentCmd() << std::endl;
							//LOG_INFO << "pro " << imp_->language_parser_.currentLine() << "---" << imp_->language_parser_.currentCmd() << std::endl;
							imp_->language_parser_.forward();
							std::unique_lock<std::mutex> lck(this->imp_->auto_mu_);
							imp_->current_line_ = imp_->language_parser_.currentLine();
							imp_->current_file_ = imp_->language_parser_.currentFile();
						}
						else if (imp_->language_parser_.currentWord() == "set")
						{
							ARIS_PRO_COUT << imp_->language_parser_.currentLine() << "---" << imp_->language_parser_.currentCmd() << std::endl;
							ARIS_LOG(aris::core::LogLvl::kError, 0, { "pro %5d---%s" }, imp_->language_parser_.currentLine(), imp_->language_parser_.currentCmd());
							try
							{
								c.calculateExpression(imp_->language_parser_.currentParamStr());
								imp_->language_parser_.forward();
								std::unique_lock<std::mutex> lck(this->imp_->auto_mu_);
								imp_->current_line_ = imp_->language_parser_.currentLine();
							}
							catch (std::exception &e)
							{
								imp_->last_error_code_ = aris::plan::Plan::PROGRAM_EXCEPTION;
								imp_->last_error_ = e.what();
								imp_->last_error_line_ = imp_->language_parser_.currentLine();
								ARIS_PRO_COUT << imp_->last_error_line_ << "---err_code:" << imp_->last_error_code_ << "  err_msg:" << imp_->last_error_ << std::endl;
								ARIS_LOG(aris::core::LogLvl::kError, 0, { "pro %5d---err_code:%s  err_msg:" }, imp_->last_error_line_, imp_->last_error_code_, imp_->last_error_);
							}
						}
						else
						{
							auto cmd = imp_->language_parser_.currentCmd();
							auto current_line = imp_->language_parser_.currentLine();
							imp_->language_parser_.forward();
							auto next_line = imp_->language_parser_.currentLine();

							auto ret = cs.executeCmdInCmdLine(cmd, [&, current_line, next_line](aris::plan::Plan &plan)->void
							{
								std::unique_lock<std::mutex> lck(imp_->auto_mu_);
								imp_->current_line_ = next_line;
							});

							ARIS_PRO_COUT << current_line << "---" << ret->cmdId() << "---" << ret->cmdString() << std::endl;
							//LOG_INFO << "pro " << current_line << "---" << ret->cmdId() << "---" << ret->cmdString() << std::endl;

							cs.waitForAllCollection();

							// 如果因为其他轨迹出错而取消 //
							if (ret->prepareRetCode() == aris::plan::Plan::PREPARE_CANCELLED || ret->executeRetCode() == aris::plan::Plan::EXECUTE_CANCELLED)
							{
								ARIS_PRO_COUT << current_line << "---" << ret->cmdId() << "---canceled" << std::endl;
								//LOG_ERROR << "pro " << current_line << "---" << ret->cmdId() << "---canceled" << std::endl;
							}
							else if (ret->executeRetCode() < 0)
							{
								imp_->last_error_code_ = ret->executeRetCode();
								imp_->last_error_ = ret->executeRetMsg();
								imp_->last_error_line_ = current_line;
								ARIS_PRO_COUT << imp_->last_error_line_ << "---" << ret->cmdId() << "---err_code:" << imp_->last_error_code_ << "  err_msg:" << imp_->last_error_ << std::endl;
								//LOG_ERROR << "pro " << imp_->last_error_line_ << "---" << ret->cmdId() << "---err_code:" << imp_->last_error_code_ << "  err_msg:" << imp_->last_error_ << std::endl;
							}

						}

						std::swap(imp_->calculator_, dynamic_cast<aris::dynamic::Model&>(aris::server::ControlServer::instance().model()).calculator());
						return send_code_and_msg(imp_->last_error_code_, imp_->last_error_);
					}
				}
				else if (param == "start")
				{
					ARIS_LOG(aris::core::LogLvl::kInfo, 0, { "pro now start" });

					if (!isAutoMode())
					{
						return send_code_and_msg(aris::plan::Plan::PROGRAM_EXCEPTION, "can not start program in manual mode");
					}
					else if (isAutoRunning())
					{
						imp_->is_pause_.store(false);
						return send_code_and_msg(0, "");
					}
					else if (lastErrorCode())
					{
						return send_code_and_msg(lastErrorCode(), lastError());
					}
					else if (!imp_->language_parser_.hasCursor())
					{
						return send_code_and_msg(-1, "please goto main or lines");
					}
					else
					{
						imp_->is_pause_.store(false);
						imp_->is_stop_.store(false);

						imp_->auto_thread_ = std::thread([&]()->void
						{
							// 交换calculator，保证每个程序开始时的变量都是之前的 //
							std::swap(imp_->calculator_, dynamic_cast<aris::dynamic::Model&>(aris::server::ControlServer::instance().model()).calculator());
							auto &c = dynamic_cast<aris::dynamic::Model&>(aris::server::ControlServer::instance().model()).calculator();
							auto &cs = aris::server::ControlServer::instance();
							std::unique_lock<std::mutex> lck(imp_->auto_mu_);
							imp_->current_line_ = imp_->language_parser_.currentLine();
							imp_->current_file_ = imp_->language_parser_.currentFile();
							lck.unlock();
							std::vector < std::pair<std::string, std::function<void(aris::plan::Plan&)>> > cmd_vec;
							std::vector <int> lines;

							for (int has_error{ 0 }; has_error == 0 && (!imp_->language_parser_.isEnd());)
							{
								if (imp_->is_stop_.load() == true)break;
								if (imp_->is_pause_.load() == true)
								{
									std::this_thread::sleep_for(std::chrono::milliseconds(1));
									continue;
								}

								// 碰到断点时才真正执行 //
								auto server_execute = [&]() ->int
								{
									auto plans = cs.executeCmdInCmdLine(cmd_vec);
									for (int i = 0; i < plans.size(); ++i)
									{
										ARIS_PRO_COUT << lines[i] << "---" << plans[i]->cmdId() << "---" << plans[i]->cmdString() << std::endl;
										//LOG_INFO << "pro " << lines[i] << "---" << plans[i]->cmdId() << "---" << plans[i]->cmdString() << std::endl;
									}
									cs.waitForAllCollection();
									for (int i = 0; i < plans.size(); ++i)
									{
										// 如果因为其他轨迹出错而取消 //
										if (plans[i]->prepareRetCode() == aris::plan::Plan::PREPARE_CANCELLED || plans[i]->executeRetCode() == aris::plan::Plan::EXECUTE_CANCELLED)
										{
											ARIS_PRO_COUT << lines[i] << "---" << plans[i]->cmdId() << "---canceled" << std::endl;
											//LOG_ERROR << "pro " << lines[i] << "---" << plans[i]->cmdId() << "---canceled" << std::endl;
										}
										else if (plans[i]->executeRetCode() < 0)
										{
											imp_->last_error_code_ = plans[i]->executeRetCode();
											imp_->last_error_ = plans[i]->executeRetMsg();
											imp_->last_error_line_ = lines[i];
											ARIS_PRO_COUT << imp_->last_error_line_ << "---" << plans[i]->cmdId() << "---err_code:" << imp_->last_error_code_ << "  err_msg:" << imp_->last_error_ << std::endl;
											//LOG_ERROR << "pro " << imp_->last_error_line_ << "---" << plans[i]->cmdId() << "---err_code:" << imp_->last_error_code_ << "  err_msg:" << imp_->last_error_ << std::endl;
											has_error = -1;
										}
									}
									cmd_vec.clear();
									lines.clear();
									plans.clear();
									return has_error;
								};

								if (imp_->language_parser_.isCurrentLineKeyWord())
								{
									if (server_execute())continue;
									ARIS_PRO_COUT << imp_->language_parser_.currentLine() << "---" << imp_->language_parser_.currentCmd() << std::endl;
									//LOG_INFO << "pro " << imp_->language_parser_.currentLine() << "---" << imp_->language_parser_.currentCmd() << std::endl;
									if (imp_->language_parser_.currentWord() == "if" || imp_->language_parser_.currentWord() == "while")
									{
										try
										{
											auto ret = c.calculateExpression(imp_->language_parser_.currentParamStr());

											if (auto ret_double = std::any_cast<double>(&ret.second))
											{
												imp_->language_parser_.forward(*ret_double != 0.0);
												std::unique_lock<std::mutex> lck(this->imp_->auto_mu_);
												imp_->current_line_ = imp_->language_parser_.currentLine();
											}
											else if (auto ret_mat = std::any_cast<aris::core::Matrix>(&ret.second))
											{
												imp_->language_parser_.forward(ret_mat->toDouble() != 0.0);
												std::unique_lock<std::mutex> lck(this->imp_->auto_mu_);
												imp_->current_line_ = imp_->language_parser_.currentLine();
											}
											else
											{
												imp_->last_error_code_ = aris::plan::Plan::PROGRAM_EXCEPTION;
												imp_->last_error_ = "invalid expresion";
												imp_->last_error_line_ = imp_->language_parser_.currentLine();
												ARIS_PRO_COUT << imp_->last_error_line_ << "---err_code:" << imp_->last_error_code_ << "  err_msg:" << imp_->last_error_ << std::endl;
												break;
											}
										}
										catch (std::exception &e)
										{
											imp_->last_error_code_ = -10;
											imp_->last_error_ = e.what();
											imp_->last_error_line_ = imp_->language_parser_.currentLine();
											ARIS_PRO_COUT << imp_->last_error_line_ << "---err_code:" << imp_->last_error_code_ << "  err_msg:" << imp_->last_error_ << std::endl;
											break;
										}
									}
									else
									{
										imp_->language_parser_.forward();
										std::unique_lock<std::mutex> lck(this->imp_->auto_mu_);
										imp_->current_line_ = imp_->language_parser_.currentLine();
										imp_->current_file_ = imp_->language_parser_.currentFile();
									}
								}
								else if (imp_->language_parser_.isCurrentLineFunction())
								{
									if (server_execute())continue;
									ARIS_PRO_COUT << imp_->language_parser_.currentLine() << "---" << imp_->language_parser_.currentCmd() << std::endl;
									//LOG_INFO << "pro " << imp_->language_parser_.currentLine() << "---" << imp_->language_parser_.currentCmd() << std::endl;
									imp_->language_parser_.forward();
									std::unique_lock<std::mutex> lck(imp_->auto_mu_);
									imp_->current_line_ = imp_->language_parser_.currentLine();
									imp_->current_file_ = imp_->language_parser_.currentFile();
								}
								else if (imp_->language_parser_.currentWord() == "set")
								{
									if (server_execute())continue;
									ARIS_PRO_COUT << imp_->language_parser_.currentLine() << "---" << imp_->language_parser_.currentCmd() << std::endl;
									//LOG_INFO << "pro " << imp_->language_parser_.currentLine() << "---" << imp_->language_parser_.currentCmd() << std::endl;
									try
									{
										c.calculateExpression(imp_->language_parser_.currentParamStr());
										imp_->language_parser_.forward();
										std::unique_lock<std::mutex> lck(this->imp_->auto_mu_);
										imp_->current_line_ = imp_->language_parser_.currentLine();
									}
									catch (std::exception &e)
									{
										imp_->last_error_code_ = aris::plan::Plan::PROGRAM_EXCEPTION;
										imp_->last_error_ = e.what();
										imp_->last_error_line_ = imp_->language_parser_.currentLine();
										ARIS_PRO_COUT << imp_->last_error_line_ << "---err_code:" << imp_->last_error_code_ << "  err_msg:" << imp_->last_error_ << std::endl;
										//LOG_ERROR << "pro " << imp_->last_error_line_ << "---err_code:" << imp_->last_error_code_ << "  err_msg:" << imp_->last_error_ << std::endl;
										has_error = -1;
									}
								}
								else
								{
									auto cmd = imp_->language_parser_.currentCmd();
									auto current_line = imp_->language_parser_.currentLine();
									imp_->language_parser_.forward();
									auto next_line = imp_->language_parser_.currentLine();

									cmd_vec.push_back(std::pair<std::string, std::function<void(aris::plan::Plan&)>>(cmd, [&, current_line, next_line](aris::plan::Plan &plan)->void
									{
										std::unique_lock<std::mutex> lck(this->imp_->auto_mu_);
										imp_->current_line_ = next_line;
									}));
									lines.push_back(current_line);
								}

								if (imp_->language_parser_.isEnd()) server_execute();
							}

							cs.waitForAllCollection();
							lck.lock();
							imp_->current_line_ = imp_->language_parser_.currentLine();
							lck.unlock();

							//std::swap(imp_->calculator_, aris::server::ControlServer::instance().model().calculator());
							ARIS_PRO_COUT << "---" << (imp_->is_stop_.load() ? "program stopped" : "program finished") << std::endl;
							//LOG_INFO << "pro " << "---" << (imp_->is_stop_.load() ? "program stopped" : "program finished") << std::endl;

							while (!imp_->auto_thread_.joinable());
							imp_->auto_thread_.detach();
						});
						return send_code_and_msg(0, "");
					}
				}
				else if (param == "stop")
				{
					if (!isAutoMode())
					{
						return send_code_and_msg(aris::plan::Plan::PROGRAM_EXCEPTION, "can not stop program in manual mode");
					}
					else
					{
						imp_->is_stop_.store(true);
						return send_code_and_msg(0, "");
					}
				}
				else if (param == "pause")
				{
					if (!isAutoMode())
					{
						return send_code_and_msg(aris::plan::Plan::PROGRAM_EXCEPTION, "can not stop program in manual mode");
					}
					else if (!isAutoRunning())
					{
						return send_code_and_msg(aris::plan::Plan::PROGRAM_EXCEPTION, "can not stop program when not running");
					}
					else
					{
						imp_->is_pause_.store(true);
						return send_code_and_msg(0, "");
					}
				}
				else if (param == "clear_error")
				{
					if (isAutoRunning())
					{
						return send_code_and_msg(aris::plan::Plan::PROGRAM_EXCEPTION, "can not clear error when running");
					}
					else
					{
						imp_->last_error_.clear();
						imp_->last_error_code_ = 0;
						imp_->last_error_line_ = 0;
						return send_code_and_msg(0, "");
					}
				}
				else
				{
					return send_code_and_msg(aris::plan::Plan::PROGRAM_EXCEPTION, "invalid program option");
				}
			}
		}
		else if (cmd == "program_file") {
			for (auto &[param, value] : params)	{
				if (param == "get")	{
					auto ret = fetchPrograms();
					return send_code_and_msg(0, ret);
				}
				else if (param == "post") {
					auto ret = createProgram(std::string(params.at("data")));
					return send_code_and_msg(0, ret);
				}
				else if (param == "put") {

					return send_code_and_msg(0, "");
				}
				else if (param == "delete") {

					return send_code_and_msg(0, "");
				}
				else if (param == "patch") {

					return send_code_and_msg(0, "");
				}
				else
				{
					return send_code_and_msg(aris::plan::Plan::PROGRAM_EXCEPTION, "invalid program option");
				}
			}
		}
		else
		{
			aris::server::ControlServer::instance().executeCmdInCmdLine(std::string(str), [send_ret](aris::plan::Plan &plan)->void
			{
				// only copy if it is a str
				if (auto js = std::any_cast<std::vector<std::pair<std::string, std::any>>>(&plan.ret()))
				{
					js->push_back(std::make_pair<std::string, std::any>("return_code", plan.executeRetCode()));
					js->push_back(std::make_pair<std::string, std::any>("return_message", std::string(plan.executeRetMsg())));
					send_ret(aris::server::parse_ret_value(*js));
				}
				else
				{
					std::vector<std::pair<std::string, std::any>> ret_js;
					ret_js.push_back(std::make_pair<std::string, std::any>("return_code", plan.executeRetCode()));
					ret_js.push_back(std::make_pair<std::string, std::any>("return_message", std::string(plan.executeRetMsg())));
					send_ret(aris::server::parse_ret_value(ret_js));
				}
			});
		}

		return 0;
	}
	ProgramMiddleware::ProgramMiddleware() :imp_(new Imp)
	{
		aris::core::Command cmd;
		aris::core::fromXmlString(cmd,
			"<Command name=\"program\">"
			"	<Param name=\"set_auto\"/>"
			"	<Param name=\"set_manual\"/>"
			"	<Param name=\"content\"/>"
			"	<Param name=\"goto\" default=\"2\"/>"
			"	<Param name=\"goto_main\"/>"
			"	<Param name=\"start\"/>"
			"	<Param name=\"pause\"/>"
			"	<Param name=\"stop\"/>"
			"	<Param name=\"clear_error\"/>"
			"	<Param name=\"forward\"/>"
			"</Command>");
		imp_->command_parser_.commandPool().push_back(cmd);

		aris::core::fromXmlString(cmd,
			"<Command name=\"program_file\">"
			"	<Param name=\"get\"/>"
			"	<Param name=\"post\"/>"
			"	<Param name=\"put\"/>"
			"	<Param name=\"delete\"/>"
			"	<Param name=\"patch\"/>"
			"</Command>");

		imp_->command_parser_.init();
	}
	ProgramMiddleware::ProgramMiddleware(ProgramMiddleware && other) = default;
	ProgramMiddleware& ProgramMiddleware::operator=(ProgramMiddleware&& other) = default;
	ProgramMiddleware::~ProgramMiddleware() = default;

	ARIS_REGISTRATION {
		aris::core::class_<CustomModule>("CustomModule");
		aris::core::class_<TransferModelController>("TransferModelController");

		typedef aris::control::Master &(ControlServer::*MasterFunc)();
		typedef aris::control::Controller &(ControlServer::*ControllerFunc)();
		typedef aris::dynamic::ModelBase &(ControlServer::*ModelFunc)();
		typedef aris::plan::PlanRoot &(ControlServer::*PlanRootFunc)();
		typedef TransferModelController& (ControlServer::* TransferModelControllerFunc)();
		typedef aris::core::PointerArray<aris::server::Interface>&(ControlServer::*InterfacePoolFunc)();
		typedef aris::server::MiddleWare &(ControlServer::*MiddleWareFunc)();
		typedef aris::server::CustomModule &(ControlServer::*CustomModuleFunc)();

		aris::core::class_<ControlServer>("ControlServer")
			.prop("model",         &ControlServer::resetModel, ModelFunc(&ControlServer::model))
			.prop("master",        &ControlServer::resetMaster, MasterFunc(&ControlServer::master))
			.prop("controller",    &ControlServer::resetController, ControllerFunc(&ControlServer::controller))
			.prop("plan_root",     &ControlServer::resetPlanRoot, PlanRootFunc(&ControlServer::planRoot))
			.prop("interface",     &ControlServer::resetInterfacePool, InterfacePoolFunc(&ControlServer::interfacePool))
			.prop("middle_ware",   &ControlServer::resetMiddleWare, MiddleWareFunc(&ControlServer::middleWare))
			.prop("model", &ControlServer::resetModel, ModelFunc(&ControlServer::model))
			.prop("master", &ControlServer::resetMaster, MasterFunc(&ControlServer::master))
			.prop("controller", &ControlServer::resetController, ControllerFunc(&ControlServer::controller))
			.prop("plan_root", &ControlServer::resetPlanRoot, PlanRootFunc(&ControlServer::planRoot))
			.prop("model_controller_transfer", &ControlServer::resetTransferModelController, TransferModelControllerFunc(&ControlServer::transferModelController))
			.prop("interface", &ControlServer::resetInterfacePool, InterfacePoolFunc(&ControlServer::interfacePool))
			.prop("middle_ware", &ControlServer::resetMiddleWare, MiddleWareFunc(&ControlServer::middleWare))
			.prop("custom_module", &ControlServer::resetCustomModule, CustomModuleFunc(&ControlServer::customModule))
			;
		
		aris::core::class_<ProgramMiddleware>("ProgramMiddleware")
			.inherit<MiddleWare>()
			;

		aris::core::class_<ScaraTransferModelController>("ScaraTransferModelController")
			.inherit<TransferModelController>()
			.prop("pitch", &ScaraTransferModelController::setPitch, &ScaraTransferModelController::pitch)
			;

		aris::core::class_<GeneralTransferModelController>("GeneralTransferModelController")
			.inherit<TransferModelController>()
			.prop("mat", &GeneralTransferModelController::setMat, &GeneralTransferModelController::mat)
			;
	}
}