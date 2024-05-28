#include "../include/read_compare_truth.h"

int read_truth_path(int ped_num) {  

    std::ifstream file("/home/linanji/src/map/src/simulator/scripts/orca_circle_crossing_5ped_1scenes_.txt");  
    if (!file.is_open()) {  
        std::cerr << "无法打开文件" << std::endl;  
        return 0;  
    }  
    // char cwd[100];
 
    // if (getcwd(cwd, sizeof(cwd)) != NULL) {
    //     printf("当前工作目录为: %s\n", cwd);
    // } else {
    //     perror("错误信息: ");
    // }
  
    int episodes = 1; 
  
    std::string line;  
    while (std::getline(file, line)) {  
        std::istringstream iss(line);  
        std::string element;  
        int index = 0;  
        int ped_id;

        std::getline(iss, element, ',');   
        std::getline(iss, element, ',');   
        ped_id = std::stoi(element);
        std::getline(iss, element, ',');
        double x=std::stod(element);
        std::getline(iss, element, ',');     
        double y=std::stod(element);
        Eigen::Vector2d point(x, y);
        switch (ped_id) {  
                            case 0: ped_0.push_back(point); break; // 假设y坐标始终为0.0f，或者你可以从下一个元素中获取  
                            case 1: ped_1.push_back(point); break;  
                            case 2: ped_2.push_back(point); break;  
                            case 3: ped_3.push_back(point); break;  
                            case 4: ped_4.push_back(point); break;  
                            default: break;  
                        }  
    }  
  
    file.close();  
    return 1;  
}



