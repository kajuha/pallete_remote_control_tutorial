# aliencontrol


## 1. Description
- ROS에서 비ROS프로그램을 구동하는 패키지

## 2. Usage
- roslaunch example:
``` XML
    <node pkg="aliencontrol" type="aliencontrol" name="aliencontrol">
        <param name="cmd" value="sh -c 'xclock'"/>
    </node>
```

## 3. Etc
- https://github.com/acschaefer/aliencontrol