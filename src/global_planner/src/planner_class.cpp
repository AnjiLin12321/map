#include "../include/planner_class.h"

namespace global
{
     //std::mutex mtx; // 互斥锁  
    Eigen::Vector3d start_point(0,0,0);
    //[0 -6 t]
   Eigen::Vector3d goal_point(0,0,0);
    //[0 6 t]

     std::vector<std::vector<common::State>>* sur_discretePoints=nullptr;

     int  obstacle_num;
double pre_time;
double deltatime;
double v_mean;
int nt;
int skip_time=0;
 double cur_time;

 double robot_r;
 double safe_dis;
 int iter_max_g;

 double goal_dis;
    int interpolation_num; 
    World_G::World_G(Eigen::Vector3d &resolution):resolution_(resolution)
    {
        lowerbound_=INF*Eigen::Vector3d::Ones();
        upperbound_=-INF*Eigen::Vector3d::Ones();
        idx_count_=Eigen::Vector3i::Zero();
    }
    World_G::~World_G()
    {
        if(has_map_)
        {
            for(int i=0;i < idx_count_(0);i++)
            {
                for(int j=0;j < idx_count_(1);j++)
                {
                    delete[] grid_map_[i][j];
                    grid_map_[i][j]=NULL;
                }
                delete[] grid_map_[i];
                grid_map_[i]=NULL;
            }
            delete[] grid_map_;
            grid_map_=NULL;
        }
    }
    void World_G::initGridMap(const Eigen::Vector3d &lowerbound,const Eigen::Vector3d &upperbound)
    {
        lowerbound_=lowerbound;
        upperbound_=upperbound;

        Eigen::Vector3d resultDouble =(upperbound_-lowerbound_).array()/resolution_.array();  
        Eigen::Vector3i resultInt = (resultDouble.array().round()).cast<int>();  
        resultInt += Eigen::Vector3i::Ones(); 
        idx_count_=resultInt;
        //((upperbound_-lowerbound_).array()/resolution_.array()).cast<int>()+Eigen::Vector3i::Ones();   //!!!
        grid_map_=new Node* **[idx_count_(0)];
        for(int i=0;i < idx_count_(0);i++)
        {
            grid_map_[i]=new Node*  *[idx_count_(1)];
            for(int j=0;j < idx_count_(1);j++)
            {
                grid_map_[i][j]=new Node*  [idx_count_(2)];
                memset(grid_map_[i][j],0,idx_count_(2)*sizeof(Node* ));
            }
        }
        has_map_=true;
    }

    bool World_G::isInsideBorder(const Eigen::Vector3i &index)
    {
         return index(0) >= 0 &&
           index(1) >= 0 &&
           index(2) >= 0 && 
           index(0) < idx_count_(0)&&
           index(1) < idx_count_(1)&&
           index(2) < idx_count_(2);
    }

    Node::Node(const Node &node)
    {
        children_=node.children_;
        parent_=node.parent_;
        position_=node.position_;
        ind_=node.ind_;
        g=node.g;
        h=node.h;
        f=node.f;
    }
    Path::Path():cost_(INF),type_(Empty){}
    Path::~Path(){}
    void AStar3D::planthread()
    {
       // std::lock_guard<std::mutex> lock(mtx); 
        using namespace std::chrono;
        system_clock::time_point current_start_time{system_clock::now()};
        system_clock::time_point next_start_time{current_start_time};
         // ROS_INFO("work_rate_ :%f",work_rate_);
        const milliseconds interval{static_cast<int>(1000.0 / work_rate_)}; // 100ms
        //ROS_INFO("interval :%d",interval);

       
        while (true) {

        


        current_start_time = system_clock::now();
        next_start_time = current_start_time + interval;
        // transform params
        start_point_=start_point;
        goal_point_=goal_point;
        init();
        dy_ob_get(sur_discretePoints);

        A3d_planner();
        //PlanCycleCallback();
        // double t_ob=(*sur_discretePoints_)[0][0].time_stamp;
        // ROS_INFO("pre  sur_discretePoints_:%f",t_ob);
        // ROS_INFO("pre  start_point:%f",start_point[2]);
        // ROS_INFO("pre  start_point_:%f",start_point_[2]);
        std::this_thread::sleep_until(next_start_time);
        // t_ob=(*sur_discretePoints_)[0][0].time_stamp;
    //    ROS_INFO("aft   sur_discretePoints_ :%f",t_ob);
    //     ROS_INFO("aft  start_point:%f",start_point[2]);
    //     ROS_INFO("aft  start_point_:%f",start_point_[2]);
       ROS_INFO("-----------------");
    }
    
        // M_Lock.lock();//加锁

		// M_Lock.unlock();//解锁

    }
    void AStar3D::dy_ob_get(std::vector<std::vector<common::State>>* sur_discretePoints){
        for(int i=0;i<obstacle_num;i++){
            for(int j=0;j<nt;j++){
                (*sur_discretePoints_)[i][j].vec_position[0]=(*sur_discretePoints)[i][j].vec_position[0];
                (*sur_discretePoints_)[i][j].vec_position[1]= (*sur_discretePoints)[i][j].vec_position[1];
                (*sur_discretePoints_)[i][j].angle= (*sur_discretePoints)[i][j].angle;
                (*sur_discretePoints_)[i][j].vec_velocity[0]= (*sur_discretePoints)[i][j].vec_velocity[0];
                (*sur_discretePoints_)[i][j].vec_velocity[1]= (*sur_discretePoints)[i][j].vec_velocity[1];
                (*sur_discretePoints_)[i][j].r= (*sur_discretePoints)[i][j].r;
                (*sur_discretePoints_)[i][j].time_stamp=(*sur_discretePoints)[i][j].time_stamp;
            }
        }
    }
    void AStar3D::A3d_planner(){
       cur_time= start_point_[2];
       ROS_INFO("cur_time %f",cur_time);
       start_point_[2]=0;
       goal_point_[2]=pre_time;
       //(*sur_discretePoints_)[0][0].time_stamp
        Eigen::Vector3i  start_ind_= world_g_->coord2index(start_point_);
        Eigen::Vector3i  goal_ind_= world_g_->coord2index(goal_point_);
        //ROS_INFO("start %d,%d,%d",start_ind_[0],start_ind_[1],start_ind_[2]);
        //ROS_INFO("goal %d,%d,%d",goal_ind_[0],goal_ind_[1],goal_ind_[2]);
        //记得 0 -skip_time 都是startpoint位置  加入closed list 记录父节点
        Node* par=nullptr;
        for(int i=0;i<skip_time;i++){
            Node* node=new Node;
            start_point_[2]=i*deltatime;
            node->position_=start_point_;
            start_ind_[2]=i;
            node->ind_=start_ind_;
            node->g=i*deltatime;
            node->h= std::abs(start_point_[0]-goal_point_[0])+ std::abs(start_point_[1]-goal_point_[1])+ weight_for_time *std::abs(start_point_[2]-goal_point_[2]);
            node->f=node->g+node->h;
            node->status= STATUS::CLOSED;
            node->parent_=par;
            par=node;

            if(!check_no_collision(*node)){
                ROS_WARN("Inilize false!");
                return;
            }

            world_g_->grid_map_[start_ind_[0]][start_ind_[1]][start_ind_[2]]=node;
        }
        Node* start_node=new Node;
        start_point_[2]=skip_time*deltatime;
        start_node->position_=start_point_;
        start_ind_[2]=skip_time;
        start_node->ind_=start_ind_;
        start_node->g=skip_time*deltatime;
        start_node->h= std::abs(start_point_[0]-goal_point_[0])+ std::abs(start_point_[1]-goal_point_[1])+ weight_for_time *std::abs(start_point_[2]-goal_point_[2]);
        start_node->f=start_node->g+start_node->h;
        start_node->status = STATUS::OPEN;
        start_node->parent_=par;
       
           
        //ROS_INFO("start_node %f",start_node->f);
        if(!check_no_collision(*start_node)){
                ROS_WARN("Inilize false!");
                return;
            }
         world_g_->grid_map_[start_ind_[0]][start_ind_[1]][start_ind_[2]]=start_node;
        openlist_.push(start_node);

        //test openlist--------------------------------------------
        // Node* t_node=new Node;
        // t_node->f=200;
        // openlist_.push( t_node);
        // ROS_INFO("size %d",openlist_.size());

        // Node* t_node1=new Node;
        // t_node1->f=100;
        // openlist_.push( t_node1);
        // ROS_INFO("size %d",openlist_.size());

        // t_node->f=80;
        // openlist_.push( t_node);
        // ROS_INFO("size %d",openlist_.size());

        //  Node* t1_node=new Node;
        // t1_node->f=30;
        // openlist_.push( t1_node);

        // Node* node1=openlist_.top();
        // openlist_.pop();
        //  Node* node2=openlist_.top();
        // openlist_.pop();
        // Node* node3=openlist_.top();
        // openlist_.pop();
        //  Node* node4=openlist_.top();
        // openlist_.pop();
        // test end-------------------------------------------------
       
        int n_expan=(2*Nring+1)*(2*Nring+1);
        std::vector<Eigen::Vector3i>  expansion_path;
        expansion_path.resize(n_expan);
        std::vector<double>  expansion_cost(n_expan);
        //std::vector<std::vector<double>> expansion_path_cost(n_expan,std::vector<double>(4,1));
        int i=0;
        for(int ii=-Nring;ii<=Nring;ii++){
            for(int jj=-Nring;jj<=Nring;jj++){
                expansion_path[i][0]=ii;
                expansion_path[i][1]=jj;
                expansion_path[i][2]=1;
                expansion_cost[i]=hypot(ii*world_g_->resolution_[0],jj*world_g_->resolution_[1])+deltatime;
                //ROS_INFO("expansion_path %d  :%d,%d,%d  cost: %f",i,expansion_path[i][0],expansion_path[i][1],expansion_path[i][2],expansion_cost[i]);
                i++;
            }
        }
        int iter=0;
        while((!openlist_.empty())){  //(iter<=iter_max_g)&&
            iter ++;
            Node* cur_node = openlist_.top();
            openlist_.pop();
           
            if(cur_node->status==STATUS::CLOSED){
                continue;
            }
            if (check_close_goal(*cur_node)){
                ROS_INFO("success");
                generatePath(cur_node);
                visPath(path_.nodes_);
                pubInterpolatedPath(path_.nodes_);
                
                return ;
            }
            
            //ROS_INFO("cur %f,%f,%f",cur_node->position_[0],cur_node->position_[1],cur_node->position_[2] );
             //visTree(*cur_node);
            cur_node->status=STATUS::CLOSED;

            Eigen::Vector3i cur_indx=cur_node->ind_;
            for(int i=0;i<n_expan;i++){
                 Eigen::Vector3i  child_ind=cur_indx+expansion_path[i];
                 //check inside border
                 if(!world_g_->isInsideBorder(child_ind)){
                    continue;
                 }
                 // check outside closelist
                //  if(! world_g_->grid_map_[cur_indx[0]][cur_indx[1]][cur_indx[2]]){
                //     continue;
                //  }
                // check collision
                if(!check_no_collision(child_ind)){
                    continue;
                 }
                 double child_g=cur_node->g+expansion_cost[i];
                Node* child_node_l=world_g_->grid_map_[child_ind[0]][child_ind[1]][child_ind[2]];
                if(!child_node_l){
                    Node* child_node=new Node;
                    child_node->parent_=cur_node;
                    child_node->ind_=child_ind;
                    Eigen::Vector3d child_pos=world_g_->index2coord(child_ind);
                    child_node->position_=child_pos;
                    child_node->g=child_g;
                    child_node->h=std::abs(child_pos[0]-goal_point_[0])+ std::abs(child_pos[1]-goal_point_[1])+ weight_for_time *std::abs(child_pos[2]-goal_point_[2]);
                    child_node->f=child_node->g+child_node->h;
                    child_node->status=STATUS::OPEN;
                    world_g_->grid_map_[child_ind[0]][child_ind[1]][child_ind[2]]=child_node;
                    openlist_.push(child_node);
                }
                else{
                    if((child_node_l->status==STATUS::OPEN)&&(child_g+0.01<child_node_l->g)){
                        child_node_l->parent_=cur_node;
                         child_node_l->g=child_g;
                         Eigen::Vector3d child_pos=child_node_l->position_;
                         child_node_l->h=std::abs(child_pos[0]-goal_point_[0])+ std::abs(child_pos[1]-goal_point_[1])+ weight_for_time *std::abs(child_pos[2]-goal_point_[2]);
                         child_node_l->f=child_node_l->g+child_node_l->h;
                         openlist_.push(child_node_l);
                    }

                }
                
            }
        }
    }

     bool AStar3D::check_no_collision(const Node & node){
        int j=node.ind_[2];
         for(int i=0;i<obstacle_num;i++){
            double ob_x=(*sur_discretePoints_)[i][j].vec_position[0];
           double ob_y= (*sur_discretePoints_)[i][j].vec_position[1];
           double robot_x=node.position_[0];
           double robot_y=node.position_[1];
           double dis=(*sur_discretePoints_)[i][j].r+robot_r+safe_dis;
           if((robot_x-ob_x)*(robot_x-ob_x)+(robot_y-ob_y)*(robot_y-ob_y)<dis*dis){
             return false;
           }
         }
          return true;

     }

     bool AStar3D::check_no_collision(const Eigen::Vector3i  &indx){
         int j=indx[2];
        Eigen::Vector3d position=world_g_->index2coord(indx);
         for(int i=0;i<obstacle_num;i++){
            double ob_x=(*sur_discretePoints_)[i][j].vec_position[0];
           double ob_y= (*sur_discretePoints_)[i][j].vec_position[1];
           double robot_x=position[0];
           double robot_y=position[1];
           double dis=(*sur_discretePoints_)[i][j].r+robot_r+safe_dis;
           if((robot_x-ob_x)*(robot_x-ob_x)+(robot_y-ob_y)*(robot_y-ob_y)<dis*dis){
             return false;
           }
         }
          return true;
     }

      bool AStar3D::check_close_goal(const Node & node){
            double robot_x=node.position_[0];
            double robot_y=node.position_[1];
            if(hypot(robot_x-goal_point_[0],robot_y-goal_point_[1])<goal_dis)
            {
                return true;
            }
            return false;
      }
     void AStar3D::init(){
        std::priority_queue<Node*, std::vector<Node*>,Node::cmp>  empty;
        std::swap(openlist_, empty);

        world_g_->clear_gridmap();

     }

     void AStar3D::visTree(){
        visualization_msgs::Marker Points, Line;
        Points.header.frame_id = Line.header.frame_id = "map";
        Points.header.stamp = Line.header.stamp = ros::Time::now();
        Points.ns = Line.ns = "Tree";
        Points.action = Line.action = visualization_msgs::Marker::ADD;
        Points.pose.orientation.w = Line.pose.orientation.w = 1.0f;
        Points.id = 0;
        Line.id = 1;

        Points.type = visualization_msgs::Marker::POINTS;
        Line.type = visualization_msgs::Marker::LINE_LIST;

        Points.scale.x = Points.scale.y = 0.05;
        Line.scale.x = 0.01;

        // Points are green and Line Strip is blue
        Points.color.g = Points.color.a = 0.5f;
        // Points.color.g = Points.color.r = 255*traversability;
        Line.color.b = Line.color.a = 0.75f;

        geometry_msgs::Point pt;
        geometry_msgs::Point parent_pt;

        for(int i=0;i<world_g_->idx_count_[0];i++){
            for(int j=0;j<world_g_->idx_count_[1];j++){
                for(int k=0;k<world_g_->idx_count_[2];k++){
                    Node *node =world_g_->grid_map_[i][j][k];
                    if(node){
                        pt.x = node->position_(0);
                        pt.y = node->position_(1);
                        pt.z = node->position_(2);
                        // std_msgs::ColorRGBA color;
                        // color.r=color.g=node->plane_->traversability;
                        // color.b=0;
                        // color.a=1;

                        // Points.colors.push_back(color);
                        Points.points.push_back(pt);
                        
                        if (node->parent_ != NULL)  // skip the root node
                        {
                            Line.points.push_back(pt);
                            parent_pt.x = node->parent_->position_(0);
                            parent_pt.y = node->parent_->position_(1);
                            parent_pt.z = node->parent_->position_(2);
                            Line.points.push_back(parent_pt);
                        }

                    }
                }
            }
        }
            
        
        tree_vis_pub_->publish(Points);
        tree_vis_pub_->publish(Line);
       
     }
    void  AStar3D::generatePath( Node * node)
    {
        path_=Path();
        while(node!=NULL)
        {
            path_.nodes_.push_back(node);
            node=node->parent_;
        }
        path_.cost_=path_.nodes_.front()->g;
        path_.type_=Path::Sub;
    }

    void AStar3D::visPath(const std::vector<Node*>& solution)
    {
    
        visualization_msgs::Marker Points, Line, Frame;
         Points.header.frame_id = Line.header.frame_id = "map";
         Points.header.stamp = Line.header.stamp = ros::Time::now();
        Line.pose.orientation.w = 1.0f;
         Points.ns = Line.ns = "Path";
        Points.id = 0;
        Line.id = 1;


        Points.type = visualization_msgs::Marker::POINTS;
        Line.type = visualization_msgs::Marker::LINE_STRIP;
       

        Points.scale.x = 0.1;
        Points.scale.y = 0.1;
        Line.scale.x = 0.1;

        Points.color.g = Points.color.a = 1.0f;
        Line.color.b = Line.color.a = 1.0f;


        if (solution.size() > 1)
        {
            std::vector<Eigen::Vector3d> pts;
            for (const auto& node : solution)
            {
                pts.push_back(node->position_); 
            }

            geometry_msgs::Point pt;
            for (const auto& coord : pts)
            {
            pt.x = coord(0);
            pt.y = coord(1);
            pt.z = coord(2);

            Points.points.push_back(pt);
            Line.points.push_back(pt);
            }
        }
         path_vis_pub_->publish(Points);
        path_vis_pub_->publish(Line);
    }

    void AStar3D::pubInterpolatedPath(const std::vector<Node*>& solution){
        std_msgs::Float32MultiArray msg;
        solution[0]->position_(2)+=cur_time;
        for (size_t i = 0; i < solution.size(); i++)
        {
            
            if (i == solution.size() - 1)
            {
            msg.data.push_back(solution[i]->position_(0));
            msg.data.push_back(solution[i]->position_(1));
            msg.data.push_back(solution[i]->position_(2));
            }
            else
            {
            solution[i+1]->position_(2)+=cur_time;
            //size_t interpolation_num = (size_t)(EuclideanDistance(solution[i + 1], solution[i]) / 0.1);
            Eigen::Vector3d diff_pt = solution[i + 1]->position_ - solution[i]->position_;
            //ROS_WARN("diff_pt %f",diff_pt[2]);
            //ROS_WARN("solution[i ]->position_ %f %f %f",solution[i ]->position_[0],solution[i ]->position_[1],solution[i ]->position_[2]);
            for (size_t j = 0; j < interpolation_num; j++)
            {
                //ROS_WARN("(float)(j / interpolation_num) %f",((float)j /(float) interpolation_num));
                Eigen::Vector3d interpt = solution[i]->position_ +  ((float)j /(float) interpolation_num) *diff_pt;
                msg.data.push_back(interpt(0));
                msg.data.push_back(interpt(1));
                msg.data.push_back(interpt(2));
            }
            }
        }
        path_inter_pub_->publish(msg);
    }

}  //