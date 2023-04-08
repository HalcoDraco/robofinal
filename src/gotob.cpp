#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose2D.h>
#include <cmath>
#include <cstring>


//asumimos mapa de 384 * 384 con una resolucion de 0.05m por celda.
//suponemos robot de 6*5 = 30cm de diametro
const int map_side_lenght = 384;
const int robot_cell_diameter = 6;

geometry_msgs::Pose2D current_pose;
geometry_msgs::Pose2D destiny;
bool grid[map_side_lenght*map_side_lenght];

//para visualizar, quitar para mejorar rendimiento
ros::Publisher pb;

class GridPoint {
    public:
        int x;
        int y;
    GridPoint(int gx, int gy) {
        x = gx;
        y = gy;
    }
};

class AStarCell {
    public:
        AStarCell * parent;
        int cell_x; //index
        int cell_y; //index
        float dist_org; //cm
        float dist_dst; //cm
        float cost; //dist_org + dist_dst

        AStarCell(int x, int y, AStarCell * parentNode) {
            cell_x = x;
            cell_y = y;
            parent = parentNode;
        }

        void calculate_dists() {
            int a = 3; //cambiar
        }
};

GridPoint translate_arr_to_grid(int index) {
    return GridPoint(index/map_side_lenght, index%map_side_lenght);
}

int translate_grid_to_arr(GridPoint point) {
    return point.x * map_side_lenght + point.y;
}

bool insideCircle(GridPoint center, GridPoint point, int diameter) {
    int dx = center.x - point.x,
        dy = center.y - point.y;
    int dist_cuadrado = dx*dx + dy*dy;
    return 4*dist_cuadrado <= diameter*diameter;
}

void updateCircleInGrid(bool grid[map_side_lenght * map_side_lenght], GridPoint center, int diameter) {
    float radius = diameter/2;
    int top = ceil(center.y - radius),
        bottom = floor(center.y + radius),
        left = ceil(center.x - radius),
        right = ceil(center.x + radius);
    
    //No se sale de los limites del mapa
    if(top < 0) {
        top = 0;
    }
    if(bottom >= map_side_lenght) {
        bottom = map_side_lenght - 1;
    }
    if(left < 0) {
        left = 0;
    }
    if(right >= map_side_lenght) {
        right = map_side_lenght - 1;
    }

    for(int j = top; j <= bottom; j++) {
        for(int i = left; i <= right; i++) {
            GridPoint p = GridPoint(i, j);
            if(insideCircle(center, p, diameter)) {
                int ind_p = translate_grid_to_arr(p);
                grid[ind_p] = true;
            }
        }
    }
}

//final_map tiene falses
void enlargeWalls(const int8_t map[map_side_lenght * map_side_lenght], bool final_map[map_side_lenght * map_side_lenght]) {

    for(int i = 0; i < map_side_lenght*map_side_lenght; i++) {
        if(map[i] > 50) {
            updateCircleInGrid(final_map, translate_arr_to_grid(i), robot_cell_diameter);
        }
    }
}

void astar() {
    //todo
}

void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    memset(grid, 0, sizeof(grid));
    enlargeWalls(&(msg->data[0]), grid);

    //mera visualizacion
    nav_msgs::OccupancyGrid mimapa;
    std::vector<int8_t> v(grid, grid + sizeof grid / sizeof grid[0]);
    for (int ind = 0; ind < map_side_lenght*map_side_lenght; ind++) {
        v[ind] = v[ind] * 100;
    }
    mimapa.data = v;
    mimapa.info.height = map_side_lenght;
    mimapa.info.width = map_side_lenght;
    mimapa.info.resolution = 0.05;
    mimapa.info.origin = msg->info.origin;
    ROS_INFO("publicando el mapa");
    pb.publish(mimapa);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "MainNode");
    ros::NodeHandle nh;
    ros::Subscriber sub_map = nh.subscribe("map", 1000, mapCallBack);
    pb = nh.advertise<nav_msgs::OccupancyGrid>("map_copia", 100);

    ros::Rate rate(50);

    nh.setParam("/turtlebot3_slam_gmapping/map_update_interval", 3.0);

    while(ros::ok()) {

        //estructura:
        // -leer mapa
        // -agrandar mapa (enlargeWalls)
        // -A*
        // -mover robot

        ros::spinOnce();
        rate.sleep();
    }

    ros::spin();

    return 0;
}