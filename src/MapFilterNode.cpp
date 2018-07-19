#include "ros/ros.h"

#include <nav_msgs/OccupancyGrid.h>

#include <vector>
#include <algorithm>
#include <utility>
#include <string.h>

using namespace std;

typedef pair<int,int> pii;

const int filterSize = 4;

ros::Subscriber sub;
ros::Publisher pub;

nav_msgs::OccupancyGrid::ConstPtr grid;
nav_msgs::OccupancyGrid newGrid;

bool ** flag;
bool ** flagRemoving;

vector<pii> toRemove;

bool ex(int y, int x) {
    return (y >= 0 && y < grid->info.height && x >= 0 && x < grid->info.width);
}

int at(int y, int x) {
    return grid->data[grid->info.width * y + x];
}

int newAt(int y, int x) {
    return newGrid.data[grid->info.width * y + x];
}


int dfs(int y, int x) {
    flag[y][x] = true;

    int sum = 1;
    if(ex(y - 1, x) && !flag[y - 1][x] && at(y - 1, x) > 0)
        sum += dfs(y - 1, x);
//    cout << flag[y + 1][x] << " " << y + 1 << " " << x << endl;
    if(ex(y + 1, x) && !flag[y + 1][x] && at(y + 1, x) > 0)
        sum += dfs(y + 1, x);
    if(ex(y, x - 1) && !flag[y][x - 1] && at(y, x - 1) > 0)
        sum += dfs(y, x - 1);
    if(ex(y, x + 1) && !flag[y][x + 1] && at(y, x + 1) > 0)
        sum += dfs(y, x + 1);

    return sum;
}

void dfsRemove(int y, int x) {
    if(flagRemoving[y][x])
        return;
    flagRemoving[y][x] = true;
    newGrid.data[grid->info.width * y + x] = 0;
    if(ex(y - 1, x) && newAt(y - 1, x) > 0)
        dfsRemove(y - 1, x);
    if(ex(y + 1, x) && newAt(y + 1, x) > 0)
        dfsRemove(y + 1, x);
    if(ex(y, x - 1) && newAt(y, x - 1) > 0)
        dfsRemove(y, x - 1);
    if(ex(y, x + 1) && newAt(y, x + 1) > 0)
        dfsRemove(y, x + 1);

}

void gridCb(const nav_msgs::OccupancyGrid::ConstPtr & _grid)
{
    cout << "123" << endl;
    grid = _grid;
    newGrid.info = grid->info;
    newGrid.header = grid->header;
    newGrid.header.stamp = ros::Time::now();
    cout << "123" << endl;
    newGrid.data = grid->data;

    cout <<  grid->info.width << " " << grid->info.height << endl;

    flag = new bool * [grid->info.height];
    flagRemoving = new bool * [grid->info.height];
    for(int y = 0; y < grid->info.height; y++) {
        flag[y] = new bool[grid->info.width];
        flagRemoving[y] = new bool[grid->info.width];
        memset(&flag[y][0], 0, sizeof(bool) * grid->info.width);
        memset(&flagRemoving[y][0], 0, sizeof(bool) * grid->info.width);
    }
    for(int y = 0; y < grid->info.height; y++) {
        for(int x = 0; x < grid->info.width; x++)
            if(at(y,x) && !flag[y][x] && dfs(y,x) <= filterSize)
                toRemove.push_back(pii(y,x));
    }
    cout << "THERE" << endl;

    for(size_t i = 0; i < toRemove.size(); i++)
        dfsRemove(toRemove[i].first, toRemove[i].second);

    cout << "THERE 2" << endl;

    pub.publish(newGrid);

    cout << "THERE 3" << endl;
    return;
//    grid = 0;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "map_filter");
    ros::NodeHandle node;

    sub = node.subscribe<nav_msgs::OccupancyGrid>("map", 1, gridCb);
    pub = node.advertise<nav_msgs::OccupancyGrid>("filtered_map", 1);

    ros::spin();

    return 0;
}