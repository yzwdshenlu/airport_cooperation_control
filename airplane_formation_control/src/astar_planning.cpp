#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <list>
#include <math.h>
using namespace std;



///////////////////////////////////////////////////////////////////////////
const int kCost1 = 10; //直移一格消耗
const int kCost2 = 14; //斜移一格消耗

struct Point
{
	int x, y; //点坐标，这里为了方便按照C++的数组来计算，x代表横排，y代表竖列
	int F, G, H; //F=G+H
	Point *parent; //parent的坐标，这里没有用指针，从而简化代码
	Point(int _x, int _y) :x(_x), y(_y), F(0), G(0), H(0), parent(NULL)  //变量初始化
	{
	}
};

class Astar
{
public:
	double robot_size = 1.0; //机器人的尺寸
	double cell_size; //地图的分辨率
    //构造函数，输入为_maze的地址
	void InitAstar(std::vector<std::vector<int>> &_maze,double resolution);
    //路径点list
	std::list<Point *> GetPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner);
private:
    //寻找路径的函数，返回的是点的指针
	Point *findPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner);
    //获取周围点
    std::vector<Point *> getSurroundPoints(const Point *point, bool isIgnoreCorner) const;
	//判断某点是否可以用于下一步判断
    bool isCanreach(const Point *point, const Point *target, bool isIgnoreCorner) const; 
	 //判断开启/关闭列表中是否包含某点
    Point *isInList(const std::list<Point *> &list, const Point *point) const;
    //从开启列表中返回F值最小的节点
	Point *getLeastFpoint(); 
	//计算FGH值
	int calcG(Point *temp_start, Point *point);
	int calcH(Point *point, Point *end);
	int calcF(Point *point);
private:
	std::vector<std::vector<int>> maze;
	std::list<Point *> openList;  //开启列表
	std::list<Point *> closeList; //关闭列表
public:
	bool maze_flag = true; //加载地图的标志位，省得多次加载浪费资源
};

// 初始化地图
void Astar::InitAstar(std::vector<std::vector<int>> &_maze,double resolution){
	maze = _maze;
	cell_size = resolution;
}

Point *Astar::getLeastFpoint() //返回最小F的点
{
	if (!openList.empty())
	{
		auto resPoint = openList.front();  //返回第一个元素
		for (auto &point : openList){
			if (point->F < resPoint->F)
			resPoint = point;
		}
		return resPoint;
	}
	return NULL;
}
Point *Astar::findPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner)
{
	openList.push_back(new Point(startPoint.x, startPoint.y)); //置入起点,拷贝开辟一个节点，内外隔离
	while (!openList.empty())
	{
		auto curPoint = getLeastFpoint(); //找到F值最小的点
		//ROS_INFO("%d,%d",curPoint->x,curPoint->y);
		openList.remove(curPoint); //从开启列表中删除
		closeList.push_back(curPoint); //放到关闭列表
		//1,找到当前周围八个格中可以通过的格子
		//ROS_INFO("改找周围点了");
		auto surroundPoints = getSurroundPoints(curPoint, isIgnoreCorner);
		for (auto &target : surroundPoints)
		{
			//ROS_INFO("开始判断周围点了");
			//2,对某一个格子，如果它不在开启列表中，加入到开启列表，设置当前格为其父节点，计算F G H
			if (!isInList(openList, target))
			{
				//ROS_INFO("判断每个目标点");
				target->parent = curPoint; //给子节点放父节点
 
				target->G = calcG(curPoint, target);
				target->H = calcH(target, &endPoint);
				target->F = calcF(target);
 
				openList.push_back(target);
			}
			//3，对某一个格子，它在开启列表中，计算G值, 如果比原来的大, 就什么都不做, 否则设置它的父节点为当前点,并更新G和F
			else
			{
				int tempG = calcG(curPoint, target);
				if (tempG < target->G)
				{
					target->parent = curPoint;
 
					target->G = tempG;
					target->F = calcF(target);
				}
			}
			{   //如果到这一步，就说明已经结束了
				Point *resPoint = isInList(openList, &endPoint);
				if (resPoint)
					return resPoint; //返回列表里的节点指针，不要用原来传入的endpoint指针，因为发生了深拷贝
			}
		}
	}
	return NULL;
}
 
std::list<Point *> Astar::GetPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner)
{
	// ROS_INFO("开始坐标:(%d,%d),结束坐标:(%d,%d)",startPoint.x,startPoint.y,endPoint.x,endPoint.y);
	Point *result = findPath(startPoint, endPoint, isIgnoreCorner);
	//ROS_INFO("找到路径了");
	std::list<Point *> path;
	//返回路径，如果没找到路径，返回空链表
	while (result)
	{
		path.push_front(result);
		result = result->parent;
	}
 
	// 清空临时开闭列表，防止重复执行GetPath导致结果异常
	openList.clear();
	closeList.clear();
	ROS_INFO("获取路径成功");
	return path;
}
 
Point *Astar::isInList(const std::list<Point *> &list, const Point *point) const
{
	//判断某个节点是否在列表中，这里不能比较指针，因为每次加入列表是新开辟的节点，只能比较坐标
	for (auto p : list){
		if (p->x == point->x&&p->y == point->y)
		return p;
	}
	return NULL;
}
 
bool Astar::isCanreach(const Point *point, const Point *target, bool isIgnoreCorner) const
{	

	//判断是否可以到达
	if (target->x<0 || target->x>maze.size()-1    //x不在地图边界内
		|| target->y<0 || target->y>maze[0].size() - 1 //y不在地图边界内
		|| maze[target->x][target->y] == 100	//障碍物
		|| (target->x == point->x&&target->y == point->y)		//与当前节点重合
		|| isInList(closeList, target)) //如果点与当前节点重合、超出地图、是障碍物、或者在关闭列表中，返回false
	{
		return false;
	}
	else
	{
		if (abs(point->x - target->x) + abs(point->y - target->y) == 1) //非斜角可以
			{
				//ROS_INFO("非斜角");
				return true;
			}
		else
		{
			//斜对角要判断是否绊住，意思就是我能不能从直线走到斜对角去
			if (maze[point->x][target->y] == 0 && maze[target->x][point->y] == 0){
				//ROS_INFO("拌不住");
				return true;
			}
				
			else{
				//ROS_INFO("绊住了");
				return isIgnoreCorner; //到这就说明是真的被绊住了，那就看用不用考虑会被绊住的条件
			}
				
		}
	}


	

}
 
std::vector<Point *> Astar::getSurroundPoints(const Point *point, bool isIgnoreCorner) const
{
	std::vector<Point *> surroundPoints;
	for (int x = point->x - 1; x <= point->x + 1; x++){
		for (int y = point->y - 1; y <= point->y + 1; y++){
			if (isCanreach(point, new Point(x, y), isIgnoreCorner))
				{
					surroundPoints.push_back(new Point(x, y));
					
				}
		}
	}
	return surroundPoints;
}

int Astar::calcG(Point *temp_start, Point *point){
	//如果x，y绝对值加起来是0那么就是一个点，如果是差1，那么就是差一格，如果不是1那就是斜着走的
	int extraG = (abs(point->x - temp_start->x) + abs(point->y - temp_start->y)) == 1 ? kCost1 : kCost2; 
	//如果是初始节点，则其父节点是空，父节点的G就为0
	int parentG = point->parent == NULL ? 0 : point->parent->G; 
	return parentG + extraG;
}

int Astar::calcH(Point *point, Point *end){
	// 使用曼哈顿距离
	return (abs(end->x - point->x)+abs(end->y - point->y)) * kCost1;

	// 使用欧几里得距离
	// double dx = end->x - point->x;
    // double dy = end->y - point->y;

    // return static_cast<int>((std::hypot(dx, dy) / std::sqrt(2)) * kCost2);
}
 
int Astar::calcF(Point *point){
	return point->G + point->H;
}
///////////////////////////////////////////////////////////////////////////
float init_pose_point[3]={0};
float goal_pose_point[3]={0};
// int *map_data_p; //地图数据指针

class MapClass{
    public:
        int origin_x = 0;
        int origin_y = 0;
        float resolution = 0;
        int width = 0;
        int height = 0;
        vector<vector<int>> map_data;
    public:
        void MapCallback(const nav_msgs::OccupancyGrid msg);
};



void MapClass::MapCallback(const nav_msgs::OccupancyGrid msg){
    origin_x = msg.info.origin.position.x;
    origin_y = msg.info.origin.position.y;
    resolution = msg.info.resolution; //像素
    width = msg.info.width;
    height = msg.info.height;

	for(int i=0;i<width;i++) map_data.push_back(vector<int>());
	for(int i=0;i<width;i++){
		for(int j=0;j<height;j++){
			map_data[i].push_back(msg.data[j*width+i]);
		}
	}
}
void InitPoseCallback(const geometry_msgs::PoseWithCovarianceStamped msg){
	init_pose_point[0] = 1;
    init_pose_point[1] = msg.pose.pose.position.x;
    init_pose_point[2] = msg.pose.pose.position.y;
	// ROS_INFO("init_pose_point[1]:%f,init_pose_point[2]:%f",init_pose_point[1],init_pose_point[2]);

}
void GoalPoseCallback(const geometry_msgs::PoseStamped msg){
	goal_pose_point[0] = 1;
    goal_pose_point[1] = msg.pose.position.x;
    goal_pose_point[2] = msg.pose.position.y;
	// ROS_INFO("goal_pose_point[1]:%f,goal_pose_point[2]:%f",goal_pose_point[1],goal_pose_point[2]);
}



int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "astar_planner");
    MapClass mapclass;//创建地图类
	Astar astar;
	bool IsGetPath =false;
	list<Point *> path_copy; 
    
	ros::NodeHandle n;
    // ros::Subscriber map_sub = n.subscribe("/map", 10, &MapClass::MapCallback, &mapclass);
	ros::Subscriber map_sub = n.subscribe("/global_costmap/costmap", 10, &MapClass::MapCallback, &mapclass); // 订阅全局膨胀地图
    ros::Subscriber init_pose_sub = n.subscribe("/initialpose",10, &InitPoseCallback);
    ros::Subscriber goal_pose_sub = n.subscribe("/move_base_simple/goal",10, &GoalPoseCallback);
	
	ros::Publisher path_pub = n.advertise<nav_msgs::Path>("path_Astar",10); 
	nav_msgs::Path pathforpub;
	pathforpub.header.frame_id = "map";
	pathforpub.header.stamp = ros::Time::now();

	ros::Rate r(1);
    while (ros::ok())
    {
        if(mapclass.height>0 && mapclass.width> 0 && astar.maze_flag)
		{
			astar.maze_flag = false;
			astar.InitAstar(mapclass.map_data,mapclass.resolution);
			// ROS_INFO("%d,%d",mapclass.width,mapclass.height);
		}
		if(init_pose_point[0]&&goal_pose_point[0]){
			
			init_pose_point[0] = 0;
			init_pose_point[1] = (init_pose_point[1] - mapclass.origin_x)/mapclass.resolution;
			init_pose_point[2] = (init_pose_point[2] - mapclass.origin_y)/mapclass.resolution;
			goal_pose_point[0] = 0;
			goal_pose_point[1] = (goal_pose_point[1] - mapclass.origin_x)/mapclass.resolution;
			goal_pose_point[2] = (goal_pose_point[2] - mapclass.origin_y)/mapclass.resolution;
			// ROS_INFO("%f,%f",init_pose_point[1],init_pose_point[2]);
			// ROS_INFO("%f,%f",goal_pose_point[1],goal_pose_point[2]);
			Point start(static_cast<int>(init_pose_point[1] + 0.5),
            	static_cast<int>(init_pose_point[2] + 0.5));
			Point end(static_cast<int>(goal_pose_point[1] + 0.5),
				static_cast<int>(goal_pose_point[2] + 0.5));
			//A星算法寻找路径
			list<Point *> path = astar.GetPath(start, end, false); //这个其实是反过来的
			pathforpub.poses.clear();
			for (auto p: path)  {
					geometry_msgs::PoseStamped this_pose_stamped;
					this_pose_stamped.header.frame_id="map";
					this_pose_stamped.header.stamp = ros::Time::now();
					// ROS_INFO("x:%d,y:%d",p->x,p->y);
					this_pose_stamped.pose.position.x = p->x*mapclass.resolution + mapclass.origin_x;
					this_pose_stamped.pose.position.y = p->y*mapclass.resolution + mapclass.origin_y;

					this_pose_stamped.pose.orientation.x = 0;
					this_pose_stamped.pose.orientation.y = 0;
					this_pose_stamped.pose.orientation.z = 0;
					this_pose_stamped.pose.orientation.w = 0;
					pathforpub.poses.push_back(this_pose_stamped);
					path_pub.publish(pathforpub);
				}
				//path_pub.publish(pathforpub);
		}
		path_pub.publish(pathforpub);
        ros::spinOnce();
		r.sleep();
    }
    return 0;
}

