
# The map configureation
RES=0.1
HEIGHT=100
L=50

# Traverse through all the maps
dirlist=$(find $1 -mindepth 1 -maxdepth 1 -type d)

for dir in $dirlist
do
  (
    # Traverse into the directories
    cd $dir
    DIRNAME="${PWD##*/}"
    FILE="$DIRNAME.world"
    PLUGINFILE="$DIRNAME"
    PLUGINFILE+="_plugin.world"
    cp $FILE $PLUGINFILE 
    
    # Edit the copied world file
    LINES=`wc -l < $PLUGINFILE`
    LINE=`expr $LINES - 1`
    CONTENT="<plugin filename="libcollision_map_creator.so" name="collision_map_creator"/>"
    sed -i "$LINE"'i'"$CONTENT" $PLUGINFILE 
    
    # Open Gazebo
    echo $DIRNAME
    
    gazebo $PLUGINFILE &
    sleep 10
    
    # Generate the png
    PNG_NAME="$DIRNAME.png"
    echo $PNG_NAME
    $TACTIC_WS/collision_map_creator_plugin/build/request_publisher "(-$L,$L)($L,$L)($L,-$L)(-$L,-$L)" $HEIGHT $RES $PNG_NAME
    
    # Generate the yaml
    YAML_NAME="$DIRNAME.yaml"
    CONTENT="image: $PNG_NAME\n"
    CONTENT+="resolution: $RES\n"
    CONTENT+="origin: [-$L, -$L, 0]\n"
    CONTENT+="occupied_thresh: 0.65\n"
    CONTENT+="free_thresh: 0.196\n"
    CONTENT+="negate: 0"
    echo -e $CONTENT > $YAML_NAME 
    
    sleep 15
    # End building png, kill gazebo
    killall -9 gazebo & killall -9 gzserver & killall -9 gzclient   

  )
done


