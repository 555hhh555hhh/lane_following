# lane following
This is Tang Hongjing's lab4 package.

# Usage

## 1. Clone the source code
  cd ~/catkin_ws/src
  
  git clone https://github.com/555hhh555hhh/lane_following.git
  
## 2. Catkin make the lane following package
  cd ..
  
  catkin_make

## 3. Add course models
   export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/catkin_ws/src/lane_following/models
   
## 4. Launch the gazebo map
   source ~/catkin_ws/devel/setup.bash
   
   roslaunch lane_following race_track.launch 

## 5. Run lane following python node
## 5.1 Run lane following part1
   
   cd ~/catkin_ws/src/lane_following/scripts/
   
   chmod +x lane_following_part1.py
   
   cd ~/catkin_ws
   
   source devel/setup.bash
   
   rosrun lane_following lane_following_part1.py

## 5.2 Run lane following part2
   
   cd ~/catkin_ws/src/lane_following/scripts/
   
   chmod +x lane_following_part2.py
   
   cd ~/catkin_ws
   
   source devel/setup.bash
   
   rosrun lane_following lane_following_part2.py

## 5.3 Run lane following part3
   
   cd ~/catkin_ws/src/lane_following/scripts/
   
   chmod +x lane_following_part3.py
   
   cd ~/catkin_ws
   
   source devel/setup.bash
   
   rosrun lane_following lane_following_part3.py

 ![image](https://github.com/zhaojieting/linefollowing/blob/main/data/demo.gif)
