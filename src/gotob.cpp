#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <cmath>
#include <cstring>
#include <vector>

//Por facilidad, asumimos un mapa de 384 * 384 con una resolucion de 0.05m por celda.
const int map_side_lenght = 384; //celdas
const int total_map_size = map_side_lenght * map_side_lenght; //celdas
const float resolution = 0.05; //m/celda
const int inv_resolution = 20; //celda/m
const int map_origin_x = -10; //m
const int map_origin_y = -10; //m
const int map_cell_zero_x = 200; //celda
const int map_cell_zero_y = 200; //celda

const int ort_cost = 10; //Coste de movimiento ortogonal
const int diag_cost = 14; //Coste de movimiento diagonal

//suponemos robot de 6*5 = 30cm de diametro
const int robot_cell_diameter = 6; //celda


//Celdas del mapa
class GridCell {
    public:
        int x;
        int y;

        GridCell() {
            x = 0;
            y = 0;
        }

        GridCell(int gx, int gy) {
            x = gx;
            y = gy;
        }

        GridCell(geometry_msgs::Pose2D pose) {
            changePos(pose);
        }

        //asume que pose esta en metros
        void changePos(geometry_msgs::Pose2D pose) {
            x = std::round(pose.x*inv_resolution + map_cell_zero_x);
            y = std::round(pose.y*inv_resolution + map_cell_zero_y);
        }
        
};

//Posicion de destino en metros
geometry_msgs::Pose2D destiny_meters; //en metros

//Posicion actual del robot en metros
geometry_msgs::Pose2D current_pose; //en metros

//Posicion de destino en celdas
GridCell destiny_cell = GridCell(); //en celdas

//Celdas del algoritmo A*
class AStarCell {
    public:

        AStarCell * parent; //Puntero a la celda padre
        int cell_x; //celda
        int cell_y; //celda
        float g_cost; //coste al origen
        float h_cost; //coste heuristico al destino (diagonal)
        float f_cost; //g_cost + h_cost

        //constructor por defecto
        AStarCell() {
            cell_x = 0;
            cell_y = 0;
            parent = NULL;
            g_cost = 0;
            h_cost = 0;
            f_cost = 0;
        }

        //constructor
        AStarCell(int x, int y, AStarCell * parentNode) {
            cell_x = x;
            cell_y = y;
            parent = parentNode;

            if(parentNode == NULL) {
                g_cost = 0;
            }else{

                //si es diagonal con respecto al padre
                if((x-parentNode->cell_x)*(x-parentNode->cell_x) + (y-parentNode->cell_y)*(y-parentNode->cell_y) != 1) {
                    g_cost = parentNode->g_cost + diag_cost;
                }else{
                    g_cost = parentNode->g_cost + ort_cost;
                }
            }
            
            //heuristica diagonal
            int dx = std::round(abs(destiny_cell.x - x));
            int dy = std::round(abs(destiny_cell.y - y));
            h_cost = (dx+dy)*ort_cost + (diag_cost-2*ort_cost) * std::min(dx, dy);
            
            //coste final
            computeFinalCost();
        }

        //computa el coste final
        void computeFinalCost() {
            f_cost = g_cost + h_cost;
        }

        //hace override del operador == para poder comparar dos objetos con == o usar algoritmos como std::find
        int operator == (const AStarCell &c)
		{
			return (c.cell_x == cell_x && c.cell_y == cell_y);
		}

        //obtiene el indice de la celda en un vector
        int getVectorIndex(std::vector<AStarCell> &vector) {
            std::vector<AStarCell>::iterator it = std::find(vector.begin(), vector.end(), *this);
            if(it != vector.end()) {
                return std::distance(vector.begin(), it);
            }else{
                return -1;
            }
        }

        //comprueba si la celda esta en un vector
        bool inVector(std::vector<AStarCell> &vector) {
            return getVectorIndex(vector) != -1;
        }

        //comprueba si el coste g es menor que el de otra celda
        bool lowerGCostThan(AStarCell &cell) {
            return g_cost < cell.g_cost;
        }

};

//Lista de celdas cerradas del algoritmo A* con direcciones de memoria fijas
AStarCell aStarFixedCellList[total_map_size];
int aStarFixedCellListIndex = 0;

//Mapa de ocupacion con paredes agrandadas
bool grid[total_map_size];

//Lista de celdas que conforman el camino
std::vector<AStarCell> path;

//Indica si se ha encontrado un camino
bool canReach = true;

//para visualizar, quitar para mejorar rendimiento
ros::Publisher mp_pb;
ros::Publisher p_pb;

//Convierte un indice de array a una celda de grid
GridCell translate_arr_to_grid(int index) {
    return GridCell(index%map_side_lenght, index/map_side_lenght);
}

//Convierte una celda de grid a un indice de array
int translate_grid_to_arr(GridCell cell) {
    return cell.y * map_side_lenght + cell.x;
}

//Comprueba si una celda esta dentro de un circulo de centro center y diametro diameter
bool insideCircle(GridCell center, GridCell cell) {
    int dx = center.x - cell.x,
        dy = center.y - cell.y;
    int dist_cuadrado = dx*dx + dy*dy;
    return 4*dist_cuadrado <= robot_cell_diameter*robot_cell_diameter;
}

//Actualiza el mapa de ocupacion creando un circulo alrededor de la celda center de diametro diameter
void updateCircleInGrid(GridCell center) {
    float radius = (float)robot_cell_diameter/2;

    //Limites del circulo
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

    //Comprueba las celdas dentro de los limites
    for(int j = top; j <= bottom; j++) {
        for(int i = left; i <= right; i++) {
            GridCell p = GridCell(i, j);

            //Si la celda esta dentro del circulo, se actualiza el mapa de ocupacion
            if(insideCircle(center, p)) {
                int ind_p = translate_grid_to_arr(p);
                grid[ind_p] = true;
            }
        }
    }
}

//Actualiza el mapa de ocupacion agrandando las paredes
void enlargeWalls(const int8_t map[total_map_size]) {

    //Para todas las celdas del mapa
    for(int i = 0; i < total_map_size; i++) {

        //Si la celda es una pared, se actualiza el mapa de ocupacion
        if(map[i] > 50) {
            updateCircleInGrid(translate_arr_to_grid(i));
        }
    }
}

//Obtiene el indice de la celda con menor f_cost en un vector
int find_lowest_cost_ind(std::vector<AStarCell> &v) {

    int ind_min = 0;

    //recorre el vector buscando la celda con menor f_cost
    for(int i = 0; i < v.size(); i++) {
        if(v[i].f_cost < v[ind_min].f_cost) {
            ind_min = i;

        //si hay empate, se elige la celda con menor h_cost
        }else if(v[i].f_cost == v[ind_min].f_cost) {
            if(v[i].h_cost < v[ind_min].h_cost) {
                ind_min = i;
            }
        }
    }
    return ind_min;
}

//Añade un vecino a la lista de vecinos si es valido
void assignValidNeighbour(int temp_x, int temp_y, AStarCell &parent, std::vector<AStarCell> &closed, std::vector<AStarCell> &neighbours) {

    AStarCell temp = AStarCell(temp_x, temp_y, &parent);

    //si la celda es valida, se añade a la lista de vecinos
    if(temp_x >= 0 && temp_x < map_side_lenght && temp_y >= 0 && temp_y < map_side_lenght && grid[translate_grid_to_arr(GridCell(temp_x, temp_y))] == false && temp.inVector(closed) == false) {
        neighbours.push_back(AStarCell(temp_x, temp_y, &parent));
    }
}

//Obtiene los vecinos validos de una celda
std::vector<AStarCell> find_neighbours(AStarCell &ini, std::vector<AStarCell> &closed) {
    
    std::vector<AStarCell> neighbours;
    int temp_x, temp_y;

    //para cada vecino (posicion adyacente)
    for (int i = 0; i <= 8; i++) {
        temp_x = ini.cell_x + (i%3) - 1;
        temp_y = ini.cell_y + (i/3) - 1;
        if(!(temp_x == ini.cell_x && temp_y == ini.cell_y)) {

            //si es valido, se añade a la lista de vecinos
            assignValidNeighbour(temp_x, temp_y, ini, closed, neighbours);
        }
    }

    return neighbours;
}

//Calcula el camino recursivamente desde el nodo inicial hasta el nodo final
void setPath(AStarCell * end) {
    if(end->parent == NULL) {
        path.push_back(*end);
    }else {
        setPath(end->parent);
        path.push_back(*end);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void astar(AStarCell start, AStarCell end) {
    // OPEN //the set of nodes to be evaluated
    std::vector<AStarCell> open;
    // CLOSED //the set of nodes already evaluated
    std::vector<AStarCell> closed;
    // add the start node to OPEN
    open.push_back(start);
    // loop

    int ind_current;
    while(true) {
        if(open.size() == 0) {
            //no hay camino
            canReach = false;
            ROS_INFO("No hay camino");
            break;
        }
    // current = node in OPEN with the lowest cost
        ind_current = find_lowest_cost_ind(open);

        //ROS_INFO("ind_current: %d", ind_current);

        aStarFixedCellList[aStarFixedCellListIndex] = open[ind_current];
        AStarCell * current = &(aStarFixedCellList[aStarFixedCellListIndex]);
        aStarFixedCellListIndex++;
        /*
        ROS_INFO("globalind: %d", globalind);
        ROS_INFO("current: %d, %d", current->cell_x, current->cell_y);
        ROS_INFO("destiny: %f, %f", destiny.x, destiny.y);
        ROS_INFO("opensize: %lu", open.size());
        */
    // remove current from OPEN
        open.erase(open.begin() + ind_current);
    // add current to CLOSED
        closed.push_back(*current);




        


/*
        grid[translate_grid_to_arr(GridCell(current->cell_x, current->cell_y))] = true;
        ROS_INFO("current: %d, %d", current->cell_x, current->cell_y);
        if(aStarFixedCellListIndex%10 == 0){
            nav_msgs::OccupancyGrid mimapa;
            std::vector<int8_t> v(grid, grid + sizeof(grid) / sizeof(grid[0]));
            for (int ind = 0; ind < map_side_lenght*map_side_lenght; ind++) {
                v[ind] = v[ind] * 100;
            }
            mimapa.data = v;
            mimapa.info.height = map_side_lenght;
            mimapa.info.width = map_side_lenght;
            mimapa.info.resolution = 0.05;
            mimapa.info.origin.position.x = -10;
            mimapa.info.origin.position.y = -10;
            
            mp_pb.publish(mimapa);
        }

*/

    // if current is the end node, path has been found
        if(*current == end) {

            //compute path
            path.clear();
            //ROS_INFO("pathsize1: %lu", path.size());
            setPath(current);
            //ROS_INFO("pathsize2: %lu", path.size());
            //ROS_INFO("globalind: %d", globalind);
            aStarFixedCellListIndex = 0;
            //ROS_INFO("end set path");
            break;
        }

    // for each neighbour of the current node
        std::vector<AStarCell> neighbours = find_neighbours(*current, closed);
        //ROS_INFO ("neighbours: %lu", neighbours.size());
        for(AStarCell neighbour : neighbours) {
            if (!neighbour.inVector(open) || neighbour.lowerGCostThan(open[neighbour.getVectorIndex(open)])){

                if(neighbour.inVector(open)) {
                    AStarCell * n = &open[neighbour.getVectorIndex(open)];
                    n->g_cost = neighbour.g_cost;
                    n->parent = current;
                    n->computeFinalCost();
                } else {
                    open.push_back(neighbour);
                }
           
            }
            
        
        }

    }
}



void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    memset(grid, 0, sizeof(grid));
    enlargeWalls(&(msg->data[0]));

    destiny_meters.x = (float)20/inv_resolution;
    destiny_meters.y = (float)60/inv_resolution;

    destiny_cell.changePos(destiny_meters);
    astar(AStarCell((0 + 200), (0 + 200), NULL), AStarCell((20 + 200), (40 + 200), NULL));
    

    //mera visualizacion
    nav_msgs::OccupancyGrid mimapa;
    std::vector<int8_t> v(grid, grid + sizeof(grid) / sizeof(grid[0]));
    for (int ind = 0; ind < map_side_lenght*map_side_lenght; ind++) {
        v[ind] = v[ind] * 100;
    }
    mimapa.data = v;
    mimapa.info.height = map_side_lenght;
    mimapa.info.width = map_side_lenght;
    mimapa.info.resolution = 0.05;
    mimapa.info.origin = msg->info.origin;
    
    mp_pb.publish(mimapa);
    ROS_INFO("publicando el mapa");

    nav_msgs::Path micamino;
    geometry_msgs::PoseStamped miarray[path.size()];
    for(int i = 0; i < path.size(); i++) {
        geometry_msgs::PoseStamped pos;
        pos.pose.position.x = (path[i].cell_x - 200) * 0.05;
        pos.pose.position.y = (path[i].cell_y - 200) * 0.05;
        miarray[i] = pos;
    }
    std::vector<geometry_msgs::PoseStamped> vec(miarray, miarray + sizeof(miarray) / sizeof(miarray[0]));
    for(geometry_msgs::PoseStamped ps : vec) {
        micamino.poses.push_back(ps);
    }
    micamino.header.frame_id = "map";
    
    p_pb.publish(micamino);
    ROS_INFO("publicando el camino");
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "MainNode");
    ros::NodeHandle nh;
    ros::Subscriber sub_map = nh.subscribe("map", 1000, mapCallBack);
    mp_pb = nh.advertise<nav_msgs::OccupancyGrid>("map_copia", 100);
    p_pb = nh.advertise<nav_msgs::Path>("path_copia", 100);
    
    

    ros::Rate rate(50);

    nh.setParam("/turtlebot3_slam_gmapping/map_update_interval", 3.0);

    while(ros::ok()/* && canReach*/) {

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

