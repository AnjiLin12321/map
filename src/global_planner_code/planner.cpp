
TrajPlannerServer::Start
--->TrajPlannerServer::MainThread() 
    --->TrajPlannerServer::PlanCycleCallback() 循环  50ms一次
    {
        //尝试从 p_input_smm_buff_（可能是一个队列或缓冲区）中取出数据，并将取出的数据赋值给 last_smm_
        p_input_smm_buff_->try_dequeue(last_smm_)
        // Update map
        auto map_ptr =
            std::make_shared<semantic_map_manager::SemanticMapManager>(last_smm_);
        //map_utils::TrajPlannerAdapter map_adapter_; TrajPlannerServer构造函数
        //p_traj_server_ =  new plan_utils::TrajPlannerServer(nh, traj_planner_work_rate, ego_id);  park.cc
        map_adapter_.set_map(map_ptr);  //=maptp, the mapinterface used in p_planner
        set_map(std::shared_ptr<IntegratedMap> map) {
            map_ = map;  // maintain a snapshop of the environment
            is_valid_ = true;
            return kSuccess;
        }
        //至此找到最新地图并存入map_adapter_
        

        if (fsm_num > 100000||CheckReplan()){  5000s
            Replan
                --->trajplan（更新动态障碍物+静态规划+优化）
        }
    }

trajplan = std::bind(&plan_manage::TrajPlanner::RunOnceParking,p_planner_);
trajplan===p_planner_的 RunOnceParking

p_planner_->set_map_interface(&map_adapter_);  
set_map_interface{
    map_itf_ = map_adapter_(外部)
}
//p_planner_   = new plan_manage::TrajPlanner(nh, ego_id, enable_urban_);  //this！ 构造时设置map_adapter_接口

//至此说明PlanCycleCallback() 循环里调用的trajplan是p_planner_的 RunOnceParking，而p_planner_已经设置接口 map_adapter_
//因此此时可以获得最新地图
map_itf_ === map_adapter_   map_ === last_smm_ 最新一个地图


动态障碍物获取
RunOnceParking--->RunMINCOParking里面
1 --->
if(map_itf_->GetMovingObsTraj(&sur_discretePoints)!=kSuccess){
      return kWrongStatus;
    }

ErrorType TrajPlannerAdapter::GetMovingObsTraj(
      std::vector<std::vector<common::State>>* sur_trajs){
  if (!is_valid_) return kWrongStatus;
  *sur_trajs = map_->movingObstraj();
  return kSuccess;
}


inline std::vector<std::vector<common::State>> movingObstraj() const {return surpoints;}
inline void set_sur_points(const std::vector<std::vector<common::State>> in){surpoints = in;}
DyObsCallback  {p_smm_->set_sur_points(sur_trajs);} //源头p_smm_在DyObsCallback函数里存数据  "/vis/parking_surround_trajs"


2 --->
ConverSurroundTrajFromPoints(sur_discretePoints,&surround_trajs);
ploy_traj_opt_->setSurroundTrajs(&surround_trajs);

3 ---> 规划
ploy_traj_opt_->OptimizeTrajectory



TO Do
动态建图
已经有/vis/parking_surround_trajs
DyObsCallback 解析/vis/parking_surround_trajs    得到  sur_trajs  不用p_smm_
sur_discretePoints==sur_trajs（GetMovingObsTraj GetMovingObsTraj  movingObstraj set_sur_points DyObsCallback）
ConverSurroundTrajFromPoints(sur_discretePoints,&surround_trajs);
//设计数据结构设计！！！
//pos x y t r
//问题：/vis/parking_surround_trajs能够传递的
plan_utils::SurroundTrajData
plan_utils::Trajectory
plan_utils::MinJerkOpt surMJO;

ploy_traj_opt_->setSurroundTrajs(&surround_trajs){
    surround_trajs_ = surround_trajs_ptr
}
//surround_trajs_可以直接用
surround_trajs_->at(sur_id).traj.getPos(pt_time);

