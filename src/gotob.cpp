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
const float heuristic_weight = 5.0; //Peso de la heuristica

//suponemos robot de 8*5 = 40cm de diametro
const int robot_cell_diameter = 8; //celda


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
        int g_cost; //coste al origen
        int h_cost; //coste heuristico al destino (diagonal)
        float f_cost; //g_cost + heuristic_weight * h_cost

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
            int dx = abs(destiny_cell.x - x);
            int dy = abs(destiny_cell.y - y);
            h_cost = (dx+dy)*ort_cost + (diag_cost-2*ort_cost) * std::min(dx, dy);
            
            //coste final
            computeFinalCost();
        }

        //computa el coste final
        void computeFinalCost() {
            f_cost = g_cost + h_cost * heuristic_weight;
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
        
        //comprueba si la celda esta en un array
        bool inArray(AStarCell * array, int array_size) {
            for(int i = 0; i < array_size; i++) {
                if(array[i] == *this) {
                    return true;
                }
            }
            return false;
        }

        //comprueba si el coste g es menor que el de otra celda
        bool lowerGCostThan(AStarCell &cell) {
            return g_cost < cell.g_cost;
        }

};

//Lista de celdas cerradas del algoritmo A* con direcciones de memoria fijas
AStarCell aStarClosed[total_map_size];

//Indice de la lista de celdas cerradas del algoritmo A*
int aStarClosedSize = 0;

//Mapa de ocupacion con paredes agrandadas
bool grid[total_map_size];

//Lista de celdas que conforman el camino
std::vector<AStarCell> path;

//Indica si se ha encontrado un camino
bool canReach = true;

//para visualizar, quitar para mejorar rendimiento
ros::Publisher map_pb;
ros::Publisher path_pb;

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
    int numfcost = 0;
    for(int i = 0; i < v.size(); i++) {
        if(v[i].f_cost < v[ind_min].f_cost) {
            ind_min = i;

        //si hay empate, se elige la celda con menor h_cost
        }else if(v[i].f_cost == v[ind_min].f_cost) {
            numfcost++;
            if(v[i].h_cost < v[ind_min].h_cost) {
                ind_min = i;
            }
        }
    }

    return ind_min;
}

//Añade un vecino a la lista de vecinos si es valido
void assignValidNeighbour(int temp_x, int temp_y, AStarCell &parent, std::vector<AStarCell> &neighbours) {

    AStarCell temp = AStarCell(temp_x, temp_y, &parent);

    //si la celda es valida, se añade a la lista de vecinos
    if(temp_x >= 0 && temp_x < map_side_lenght && temp_y >= 0 && temp_y < map_side_lenght && grid[translate_grid_to_arr(GridCell(temp_x, temp_y))] == false && temp.inArray(aStarClosed, aStarClosedSize) == false) {
        neighbours.push_back(AStarCell(temp_x, temp_y, &parent));
    }
}

//Obtiene los vecinos validos de una celda
std::vector<AStarCell> find_neighbours(AStarCell &ini) {
    
    std::vector<AStarCell> neighbours;
    int temp_x, temp_y;

    //para cada vecino (posicion adyacente)
    for (int i = 0; i <= 8; i++) {
        temp_x = ini.cell_x + (i%3) - 1;
        temp_y = ini.cell_y + (i/3) - 1;
        if(!(temp_x == ini.cell_x && temp_y == ini.cell_y)) {

            //si es valido, se añade a la lista de vecinos
            assignValidNeighbour(temp_x, temp_y, ini, neighbours);
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

//Publica el mapa de ocupacion de paredes agrandadas
void mapPublication() {
    nav_msgs::OccupancyGrid mp;
    std::vector<int8_t> v(grid, grid + total_map_size);
    for (int ind = 0; ind < total_map_size; ind++) {
        v[ind] = v[ind] * 100;
    }
    mp.data = v;
    mp.info.height = map_side_lenght;
    mp.info.width = map_side_lenght;
    mp.info.resolution = resolution;
    mp.info.origin.position.x = map_origin_x;
    mp.info.origin.position.y = map_origin_y;
    
    map_pb.publish(mp);
}

void pathPublication() {
    nav_msgs::Path pth;

    for(int i = 0; i < path.size(); i++) {
        geometry_msgs::PoseStamped positionStamped;
        positionStamped.pose.position.x = (path[i].cell_x - map_cell_zero_x)*resolution;
        positionStamped.pose.position.y = (path[i].cell_y - map_cell_zero_y)*resolution;
        pth.poses.push_back(positionStamped);
    }

    pth.header.frame_id = "map";
    
    path_pb.publish(pth);
}

//Visualiza el algoritmo A* en el mapa de ocupacion, solo para pruebas
void aStarVisualization(int x, int y) {

    const int intervalo_pub = 10;

    grid[translate_grid_to_arr(GridCell(x, y))] = true;

    if(aStarClosedSize % intervalo_pub == 0){
        mapPublication();        
    }
}

//Encuentra la celda libre mas cercana a la celda inicial
void findNearestFreeCell(std::vector<AStarCell> &open, AStarCell &start) {

    //Busca en 8 direcciones
    GridCell directions[8] = {GridCell(1, 0), GridCell(1, 1), GridCell(0, 1), GridCell(-1, 1), GridCell(-1, 0), GridCell(-1, -1), GridCell(0, -1), GridCell(1, -1)};
    int dist = 1;
    bool found = false;
    int i;
    while(!found) {
        for(i = 0; i < 8; i++) {
            if(grid[translate_grid_to_arr(GridCell(start.cell_x + dist*directions[i].x, start.cell_y + dist*directions[i].y))] == false) {
                found = true;
                break;
            }
        }
        if(!found){
            dist++;
        }
    }

    aStarClosed[aStarClosedSize] = start; 
    AStarCell * parent = &aStarClosed[aStarClosedSize];
    aStarClosedSize++;

    for(int j = 1; j < dist; j++) {
        aStarClosed[aStarClosedSize] = AStarCell(start.cell_x + j*directions[i].x, start.cell_y + j*directions[i].y, parent);
        parent = &aStarClosed[aStarClosedSize];
        aStarClosedSize++;     
    }
    open.push_back(AStarCell(start.cell_x + dist*directions[i].x, start.cell_y + dist*directions[i].y, parent));
    

}

void astar(AStarCell start, AStarCell end) {

    if(grid[translate_grid_to_arr(GridCell(end.cell_x, end.cell_y))] == true) {
        ROS_INFO("No hay camino o el destino esta demasiado cerca de una pared");
        canReach = false;
        return;
    }

    //Vector open, que contiene los nodos que se van a explorar (los de la frontera)
    std::vector<AStarCell> open;

    //Vaciar el vector de nodos cerrados
    aStarClosedSize = 0;

    if(grid[translate_grid_to_arr(GridCell(start.cell_x, start.cell_y))] == true) {
        //Si el nodo inicial esta en una pared, se busca el nodo libre mas cercano
        findNearestFreeCell(open, start);
    }else {
        //Se añade el nodo inicial a la lista de nodos a explorar
        open.push_back(start);
    }
    

    //Indice del nodo actual, definido fuera del bucle por eficiencia
    int ind_current;

    //Mientras no se llegue al destino y no se agoten los nodos a explorar
    while(true) {
        //Si no quedan nodos por explorar, no hay camino
        if(open.size() == 0) {
            //no hay camino
            canReach = false;
            ROS_INFO("No hay camino");
            break;
        }

        //Se obtiene el indice del nodo con menor f_cost de la lista de nodos a explorar
        ind_current = find_lowest_cost_ind(open);

        //Se copia el nodo actual a la lista fija de nodos cerrados para que no cambie su direccion de memoria a lo largo del algoritmo
        aStarClosed[aStarClosedSize] = open[ind_current];

        //Se obtiene la direccion de memoria del nodo actual
        AStarCell * current = &(aStarClosed[aStarClosedSize]);

        //Se visualiza el algoritmo A* en el mapa de ocupacion, solo para pruebas
        //aStarVisualization(current->cell_x, current->cell_y);

        //Se incrementa el indice de la lista fija de nodos para que no sobreescriba el nodo actual
        aStarClosedSize++;

        //Se elimina el nodo actual de la lista de nodos a explorar
        open.erase(open.begin() + ind_current);

        //Si el nodo actual es el destino, se ha encontrado el camino
        if(*current == end) {

            //Se calcula y asigna el camino
            path.clear();
            setPath(current);
            break;
        }

        //Se obtienen los vecinos del nodo actual
        std::vector<AStarCell> neighbours = find_neighbours(*current);

        for(AStarCell neighbour : neighbours) {

            //Encuentra la posicion del vecino en el vector open, si no esta devuelve -1
            int ind_in_open = neighbour.getVectorIndex(open);

            //Si el vecino no esta en la lista de nodos a explorar
            if(ind_in_open == -1) {

                //Se añade a la lista de nodos a explorar
                open.push_back(neighbour);

            //Si el vecino ya esta en la lista de nodos a explorar, pero tiene un g_cost inferior al que tiene ahora
            } else if (neighbour.lowerGCostThan(open[ind_in_open])){

                //Se actualizan los valores
                open[ind_in_open].g_cost = neighbour.g_cost;
                open[ind_in_open].parent = current;
                open[ind_in_open].computeFinalCost();

            }   
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////

void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    memset(grid, 0, sizeof(grid));
    enlargeWalls(&(msg->data[0]));

    destiny_meters.x = (float)20/inv_resolution;
    destiny_meters.y = (float)60/inv_resolution;

    destiny_cell.changePos(destiny_meters);
    astar(AStarCell((0 + 200), (0 + 200), NULL), AStarCell((20 + 200), (40 + 200), NULL));
    
    //mera visualizacion
    mapPublication();
    pathPublication();
}

void moveRobot() {
    //TODO
}

int main(int argc, char **argv) {

    //Inicializacion de ROS
    ros::init(argc, argv, "MainNode");

    ros::NodeHandle nh;

    //Subscriber al mapa generado por gmapping
    ros::Subscriber sub_map = nh.subscribe("map", 1000, mapCallBack);

    //Publisher del mapa ampliado
    map_pb = nh.advertise<nav_msgs::OccupancyGrid>("big_wall_map", 100);

    //Publisher del camino
    path_pb = nh.advertise<nav_msgs::Path>("robot_path", 100);
    

    ros::Rate rate(50);

    nh.setParam("/turtlebot3_slam_gmapping/map_update_interval", 2.0);

    //Recibe la informacion entrante de forma asincrona, sustituye a ros::spin()
    ros::AsyncSpinner spinner(0);
    spinner.start();

    while(ros::ok() && canReach) {

        //estructura:
        // -leer mapa
        // -agrandar mapa (enlargeWalls)
        // -A*
        // -mover robot

        rate.sleep();
    }

    ROS_INFO("Fin del programa");

    ros::waitForShutdown();

    return 0;
}