# SLAM

Map -> Odom -> base_link

### /odom
* nav_msgs/Odometry
* same position info as odom -> base_link TF
* velocity
* covariance

### /map
* grid map occupancy data
* nav_msgs/OccupancyGrid

### Utilizar slam online que significa que nao precisa de um mapa previo

### AMCL
saved map file -> map_server -> /map and /scan -> amcl -> /map -> odom -> base_link   