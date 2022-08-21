﻿#ifndef ARIS_SERVER_CONTROL_SERVER_H_
#define ARIS_SERVER_CONTROL_SERVER_H_

#include <string>
#include <sstream>
#include <map>
#include <memory>
#include <future>

#include <aris/core/core.hpp>
#include <aris/control/control.hpp>
#include <aris/sensor/sensor.hpp>
#include <aris/dynamic/dynamic.hpp>
#include <aris/plan/plan.hpp>

#include "aris/server/interface.hpp"
#include "aris/server/middle_ware.hpp"

namespace aris::server
{
	class ARIS_API CustomModule {
	public:
		CustomModule() = default;
		virtual ~CustomModule() = default;
	};

	class TerminalInterface : public Interface {
	public:
		auto virtual open()->void override { is_open_ = true; }
		auto virtual close()->void override { is_open_ = false; }
		auto virtual isConnected() const->bool override { return is_open_; }

	private:
		bool is_open_{true};

	public:
		TerminalInterface(const std::string &name = "Terminal") : Interface(name) {}
	};

	class ARIS_API TransferModelController {
	public:
		auto virtual updateDataController2Model(
			const std::vector<std::uint64_t>& options, 
			const aris::control::Controller *controller, 
			aris::dynamic::ModelBase *model)->void;
		auto virtual updateDataModel2Controller(
			const std::vector<std::uint64_t>& options, 
			const aris::dynamic::ModelBase* model,
			aris::control::Controller* controller)->void;
	};

	class ARIS_API ScaraTransferModelController:public TransferModelController {
	public:
		auto updateDataController2Model(
			const std::vector<std::uint64_t>& options,
			const aris::control::Controller* controller,
			aris::dynamic::ModelBase* model)->void override
		{
			for (std::size_t i = 0; i < controller->motorPool().size(); ++i) {
				auto& cm = controller->motorPool()[i];
				if ((options[i] & aris::plan::Plan::UPDATE_MODEL_POS_FROM_CONTROLLER))
					model->setInputPosAt(cm.targetPos(), i);
				if ((options[i] & aris::plan::Plan::UPDATE_MODEL_VEL_FROM_CONTROLLER))
					model->setInputVelAt(cm.targetVel(), i);
			}

			if (options[2] & aris::plan::Plan::UPDATE_MODEL_POS_FROM_CONTROLLER)
				model->setInputPosAt(controller->motorPool()[2].targetPos() + pitch_ / 2 / aris::PI * model->inputPosAt(3), 2);
		}
		auto updateDataModel2Controller(
			const std::vector<std::uint64_t>& options,
			const aris::dynamic::ModelBase* model,
			aris::control::Controller* controller)->void override
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

			if(options[2] & aris::plan::Plan::USE_TARGET_POS)
				controller->motorPool()[2].setTargetPos(model->inputPosAt(2) - pitch_/2/aris::PI * model->inputPosAt(3));
		}

		ScaraTransferModelController(double pitch = 0.0) {
			pitch_ = pitch;
		}

		double pitch_;
	};

	// 当 executeCmd(str, callback) 时，系统内的执行流程如下：
	// 1.   parse cmd list
	//   1.1 ---   all success : goto 2   ---err_code : SUCCESS              ---err_msg : ""                ---plan : cloned
	//   1.2 ---   any throw   : goto 5a  ---err_code :                      ---err_msg :                   ---plan : 
	//                              cmds_before_error : PREPARE_CANCELLED               : ""                        : cloned
	//                                   cmd_at_error : PARSE_EXCEPTION                 : exception.what()          : default
	//                               cmds_after_error :                                 : ""                        : default
	// 
	// 2.   prepare cmd list
	//   2.1 ---   all success : goto 3   ---err_code : SUCCESS              ---err_msg : prepare_ret_msg   ---plan : prepared
	//   2.2 ---  ret code < 0 : goto 5   ---err_code :                      ---err_msg :                   ---plan :
	//                              cmds_before_error : EXECUTE_CANCELLED               : prepare_ret_msg           : prepared
	//                                   cmd_at_error : ret_code                        : prepare_ret_msg           : cloned
	//                               cmds_after_error : PREPARE_CANCELLED               : ""                        : cloned        
	//   2.3 ---   any throw   : goto 5   ---err_code :                      ---err_msg :                   ---plan :
	//                              cmds_before_error : EXECUTE_CANCELLED               : prepare_ret_msg           : prepared
	//                                   cmd_at_error : ret_code                        : exception.what()          : cloned
	//                               cmds_after_error : PREPARE_CANCELLED               : ""                        : cloned  
	//     
	// 3.   before execute in Nrt
	//   3.1 ---   all success : goto 4   ---err_code : SUCCESS              ---err_msg : prepare_ret_msg   ---plan : prepared
	//   3.2 ---    any error  : goto 5   ---err_code :                      ---err_msg :                   ---plan :
	//                                  need run cmds : SERVER_IN_ERROR                 : "server in error..."      : prepared
	//                                                : SERVER_NOT_STARTED              : "server not started..."   : prepared
	//                                                : COMMAND_POOL_IS_FULL            : "cmd pool is full..."     : prepared
	//                              non-need run cmds : SUCCESS              ---err_msg : prepare_ret_msg   ---plan : prepared
	// 
	// 4.   execute in RT
	//   4.1 ---   all success : goto 5   ---err_code :                      ---err_msg :                   ---plan : 
	//                                  need run cmds : SUCCESS                         : execute_ret_msg           : executed
	//                              non-need run cmds : SUCCESS                         : prepare_ret_msg   ---plan : prepared
	//   4.2 ---    ret < 0    : goto 5   ---err_code :                      ---err_msg :                   ---plan :
	//                           need run cmds before : SUCCESS                         : execute_ret_msg           : executed
	//                            need run cmds error : err_code                        : execute_ret_msg           : executed
	//                            need run cmds after : EXECUTE_CANCELLED               : prepare_ret_msg           : prepared
	//                              non-need run cmds : SUCCESS              ---err_msg : prepare_ret_msg   ---plan : prepared
	// 
	// 5.   collect cmd list   : goto 6
	// 
	// 6.   post callback      : goto end
	// end
	class ARIS_API ControlServer{
	public:
		using PreCallback = std::add_pointer<void(ControlServer&)>::type;
		using PostCallback = std::add_pointer<void(ControlServer&)>::type;
		using ExecuteCmdCallback = std::function<void(aris::plan::Plan&)>;

		static auto instance()noexcept->ControlServer &;

		// model base //
		template<typename T = aris::dynamic::Model, typename... Args>
		auto makeModel(Args&&... args)noexcept->void { this->resetModel(new T(std::forward<Args>(args)...)); }
		auto resetModel(dynamic::ModelBase* model)->void;
		auto model()->dynamic::ModelBase&;
		auto model()const ->const dynamic::ModelBase& { return const_cast<ControlServer*>(this)->model(); }

		// master //
		template<typename T = aris::control::Master, typename... Args>
		auto makeMaster(Args&&... args)noexcept->void { this->resetMaster(new T(std::forward<Args>(args)...)); }
		auto resetMaster(control::Master *master)->void;
		auto master()->control::Master&;
		auto master()const ->const control::Master& { return const_cast<ControlServer *>(this)->master(); }

		// controller //
		template<typename T = aris::control::Controller, typename... Args>
		auto makeController(Args&&... args)noexcept->void { this->resetController(new T(std::forward<Args>(args)...)); }
		auto resetController(control::Controller *controller)->void;
		auto controller()->control::Controller&;
		auto controller()const ->const control::Controller& { return const_cast<ControlServer *>(this)->controller(); }

		// plans //
		template<typename T = aris::plan::PlanRoot, typename... Args>
		auto makePlanRoot(Args&&... args)noexcept->void { this->resetPlanRoot(new T(std::forward<Args>(args)...)); }
		auto resetPlanRoot(plan::PlanRoot *sensor_root)->void;
		auto planRoot()->plan::PlanRoot&;
		auto planRoot()const ->const plan::PlanRoot& { return const_cast<ControlServer *>(this)->planRoot(); }

		// model controller transfer
		template<typename T = aris::plan::PlanRoot, typename... Args>
		auto makeTransferModelController(Args&&... args)noexcept->void { this->resetTransferModelController(new T(std::forward<Args>(args)...)); }
		auto resetTransferModelController(TransferModelController*)->void;
		auto transferModelController()->TransferModelController&;
		auto transferModelController()const ->const TransferModelController& { return const_cast<ControlServer*>(this)->transferModelController(); }

		// interfaces //
		auto resetInterfacePool(aris::core::PointerArray<aris::server::Interface> *pool)->void;
		auto interfacePool()->aris::core::PointerArray<aris::server::Interface>&;
		auto interfacePool()const ->const aris::core::PointerArray<aris::server::Interface>& { return const_cast<ControlServer *>(this)->interfacePool(); }

		// middleware //
		auto resetMiddleWare(server::MiddleWare *middle_ware)->void;
		auto middleWare()->MiddleWare&;
		auto middleWare()const ->const MiddleWare& { return const_cast<ControlServer *>(this)->middleWare(); }

		// custom module //
		auto resetCustomModule(server::CustomModule *custom_module)->void;
		auto customModule()->CustomModule&;
		auto customModule()const ->const CustomModule& { return const_cast<ControlServer *>(this)->customModule(); }

		// rt error handler //
		// p can be nullptr, means idle
		auto setRtErrorCallback(std::function<void(aris::plan::Plan *p, int error_num, const char *error_msg)>)noexcept->void;

		// operation in RT & NRT context //
		auto updateDataController2Model(const std::vector<std::uint64_t>& options)noexcept->void;
		auto updateDataModel2Controller(const std::vector<std::uint64_t>& options)noexcept->void;
		auto setRtPlanPreCallback(PreCallback pre_callback)noexcept->void;
		auto setRtPlanPostCallback(PostCallback post_callback)noexcept->void;
		auto running()noexcept->bool;
		auto globalCount()noexcept->std::int64_t;
		auto currentExecutePlanRt()noexcept->aris::plan::Plan *;
		auto globalMotionCheckOption()noexcept->std::uint64_t*;// 全局的初始检查选项，也是每个 plan 的初始检查选项，但不影响空闲时候的检查
		auto idleMotionCheckOption()noexcept->std::uint64_t*;
		auto setAutoLogActive(bool auto_log)noexcept->void; // 为每个 plan 自动创建日志
		auto autoLogActive()noexcept->bool;




		// operation in NRT context //
		auto init()->void;
		auto start()->void;
		auto stop()->void;
		auto open()->void;
		auto close()->void;
		auto runCmdLine()->void;

		auto setDefaultChanel(int chanel)noexcept->void;
		auto defaultChanel()const noexcept->int;

		// chanel 为 -1 时，会使用defaultChanel
		auto executeCmd(std::vector<std::pair<std::string, ExecuteCmdCallback>>, int chanel = -1)->std::vector<std::shared_ptr<aris::plan::Plan>>;
		auto executeCmd(std::string cmd_str, ExecuteCmdCallback callback = nullptr, int chanel = -1)->std::shared_ptr<aris::plan::Plan>;
		auto executeCmdInCmdLine(std::vector<std::pair<std::string, ExecuteCmdCallback>>, int chanel = -1)->std::vector<std::shared_ptr<aris::plan::Plan>>;
		auto executeCmdInCmdLine(std::string cmd_string, ExecuteCmdCallback callback = nullptr, int chanel = -1)->std::shared_ptr<aris::plan::Plan>;
		auto currentExecutePlan(int chanel = -1)->std::shared_ptr<aris::plan::Plan>;
		auto waitForAllExecution(int chanel = -1)->void;
		auto waitForAllCollection(int chanel = -1)->void;
		auto getRtData(const std::function<void(ControlServer&, const aris::plan::Plan *target, std::any&)>& get_func, std::any& data)->void;
		auto setErrorCode(std::int32_t err_code, const char *err_msg = nullptr)noexcept->void;
		auto errorCode()const noexcept->int;
		auto errorMsg()const noexcept->const char *;
		auto clearError()->void;

	private:
		~ControlServer();
		ControlServer();
		ControlServer(const ControlServer &) = delete;
		ControlServer &operator=(const ControlServer &) = delete;

	private:
		struct Imp;
		std::unique_ptr<Imp> imp_;
	};

	class ARIS_API ProgramMiddleware : public MiddleWare{
	public:
		auto isAutoMode()->bool;
		auto isAutoRunning()->bool;
		auto isAutoPaused()->bool;
		auto isAutoStopped()->bool;
		auto lastError()->std::string;
		auto lastErrorCode()->int;
		auto lastErrorLine()->int;
		auto currentFileLine()->std::tuple<std::string, int>;
		auto executeCmd(std::string_view str, std::function<void(std::string)> send_ret, Interface *interface)->int override;
		~ProgramMiddleware();

		ProgramMiddleware();
		ProgramMiddleware(ProgramMiddleware && other);
		ProgramMiddleware& operator=(ProgramMiddleware&& other);

	private:
		struct Imp;
		std::unique_ptr<Imp> imp_;
	};



}

#endif

